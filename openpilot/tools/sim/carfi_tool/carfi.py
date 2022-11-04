#!/usr/bin/env python3
import fa_lib
from functools import partial
import analysis
import argparse
import subprocess
from argparse import RawTextHelpFormatter
from ast import parse
from distutils.debug import DEBUG
import re
from tracemalloc import start
from numpy import random
import math
import threading
import time
import os
import sys
import json
from multiprocessing import Process, Queue
from typing import Any
from sys import argv
import carla  # pylint: disable=import-error
import numpy as np
import pandas as pd
import pyopencl as cl
import pyopencl.array as cl_array
from can import can_function
import cereal.messaging as messaging
from cereal import log
from cereal.visionipc.visionipc_pyx import VisionIpcServer, VisionStreamType  # pylint: disable=no-name-in-module, import-error
from common.basedir import BASEDIR
from common.numpy_fast import clip
from common.params import Params
from common.realtime import DT_DMON, Ratekeeper
from selfdrive.car.honda.values import CruiseButtons
from selfdrive.test.helpers import set_params_enabled
from agents.navigation.basic_agent import BasicAgent
from agents.navigation.behavior_agent import BehaviorAgent
import logging
from pythonjsonlogger import jsonlogger
from datetime import datetime

logger = logging.getLogger()

logHandler = logging.FileHandler('./log/carfi_'+datetime.utcnow().strftime('%Y-%m-%d_%H_%M')+'.log')


class CustomJsonFormatter(jsonlogger.JsonFormatter):
    def add_fields(self, log_record, record, message_dict):
        super(CustomJsonFormatter, self).add_fields(
            log_record, record, message_dict)
        if not log_record.get('DateTime'):
            # this doesn't use record.created, so it is slightly off
            now = datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S')
            log_record['DateTime'] = now
        if log_record.get('Level'):
            log_record['Level'] = log_record['level'].upper()
        else:
            log_record['Level'] = record.levelname


formatter = CustomJsonFormatter('%(DateTime)s %(Level)s')

logHandler.setFormatter(formatter)
logger.addHandler(logHandler)
logger.setLevel(logging.INFO)

random.seed(123)

W, H = 1928, 1208
REPEAT_COUNTER = 5
PRINT_DECIMATION = 100
STEER_RATIO = 15.

pm = messaging.PubMaster(['roadCameraState', 'wideRoadCameraState',
                         'sensorEvents', 'can', "gpsLocationExternal"])
sm = messaging.SubMaster(['carControl', 'controlsState'])


class VehicleState:
    def __init__(self):
        self.speed = 0
        self.angle = 0
        self.bearing_deg = 0.0
        self.vel = carla.Vector3D()
        self.cruise_button = 0
        self.is_engaged = False
        self.ignition = True
        self.start_delay = 0.0


def steer_rate_limit(old, new):
    # Rate limiting to 0.5 degrees per step
    limit = 0.5
    if new > old + limit:
        return old + limit
    elif new < old - limit:
        return old - limit
    else:
        return new


class Camerad:
    def __init__(self, world, g_run, vehicle_state, fault):
        self.world = world
        self.g_run = g_run
        self.vehicle_state = vehicle_state
        self.fault = fault
        self.frame_road_id = 0
        self.frame_wide_id = 0
        self.vipc_server = VisionIpcServer("camerad")

        # TODO: remove RGB buffers once the last RGB vipc subscriber is removed
        self.vipc_server.create_buffers(
            VisionStreamType.VISION_STREAM_RGB_ROAD, 4, True, W, H)
        self.vipc_server.create_buffers(
            VisionStreamType.VISION_STREAM_ROAD, 5, False, W, H)

        self.vipc_server.create_buffers(
            VisionStreamType.VISION_STREAM_RGB_WIDE_ROAD, 4, True, W, H)
        self.vipc_server.create_buffers(
            VisionStreamType.VISION_STREAM_WIDE_ROAD, 5, False, W, H)
        self.vipc_server.start_listener()

        # set up for pyopencl rgb to yuv conversion
        self.ctx = cl.create_some_context()
        self.queue = cl.CommandQueue(self.ctx)
        cl_arg = f" -DHEIGHT={H} -DWIDTH={W} -DRGB_STRIDE={W*3} -DUV_WIDTH={W // 2} -DUV_HEIGHT={H // 2} -DRGB_SIZE={W * H} -DCL_DEBUG "

        # TODO: move rgb_to_yuv.cl to local dir once the frame stream camera is removed
        kernel_fn = os.path.join(BASEDIR, "selfdrive",
                                 "camerad", "transforms", "rgb_to_yuv.cl")
        prg = cl.Program(self.ctx, open(kernel_fn).read()).build(cl_arg)
        self.krnl = prg.rgb_to_yuv
        self.Wdiv4 = W // 4 if (W % 4 == 0) else (W + (4 - W % 4)) // 4
        self.Hdiv4 = H // 4 if (H % 4 == 0) else (H + (4 - H % 4)) // 4

    def cam_callback_road(self, image):
        if not self.g_run and self.vehicle_state.start_delay > 0:
            sim_time = round(
                self.world.get_snapshot().timestamp.elapsed_seconds, 3)

            if sim_time >= self.fault['activation_time'] and sim_time < self.fault['deactivation_time']:
                if self.fault['impl'] == 'server':
                    return
                image = getattr(fa_lib, self.fault['type'])(image)

        self._cam_callback(image, self.frame_road_id, 'roadCameraState',
                           VisionStreamType.VISION_STREAM_RGB_ROAD, VisionStreamType.VISION_STREAM_ROAD)
        self.frame_road_id += 1

    def cam_callback_road_faulty(self, image):
        if not self.g_run:
            if self.vehicle_state.start_delay == 0.0:
                return

            sim_time = round(
                self.world.get_snapshot().timestamp.elapsed_seconds, 3)

            if sim_time >= self.fault['deactivation_time'] or sim_time < self.fault['activation_time']:
                return

        self._cam_callback(image, self.frame_road_id, 'roadCameraState',
                           VisionStreamType.VISION_STREAM_RGB_ROAD, VisionStreamType.VISION_STREAM_ROAD)
        self.frame_road_id += 1

    def cam_callback_wide_road(self, image):
        self._cam_callback(image, self.frame_wide_id, 'wideRoadCameraState',
                           VisionStreamType.VISION_STREAM_RGB_WIDE_ROAD, VisionStreamType.VISION_STREAM_WIDE_ROAD)
        self.frame_wide_id += 1

    def _cam_callback(self, image, frame_id, pub_type, rgb_type, yuv_type):
        img = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        img = np.reshape(img, (H, W, 4))
        img = img[:, :, [0, 1, 2]].copy()

        # convert RGB frame to YUV
        rgb = np.reshape(img, (H, W * 3))
        rgb_cl = cl_array.to_device(self.queue, rgb)
        yuv_cl = cl_array.empty_like(rgb_cl)
        self.krnl(self.queue, (np.int32(self.Wdiv4), np.int32(
            self.Hdiv4)), None, rgb_cl.data, yuv_cl.data).wait()
        yuv = np.resize(yuv_cl.get(), np.int32(rgb.size / 2))
        eof = int(frame_id * 0.05 * 1e9)

        # TODO: remove RGB send once the last RGB vipc subscriber is removed
        self.vipc_server.send(rgb_type, img.tobytes(), frame_id, eof, eof)
        self.vipc_server.send(yuv_type, yuv.data.tobytes(), frame_id, eof, eof)

        dat = messaging.new_message(pub_type)
        msg = {
            "frameId": image.frame,
            "transform": [1.0, 0.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0]
        }
        setattr(dat, pub_type, msg)
        pm.send(pub_type, dat)


def imu_callback(imu, vehicle_state):
    vehicle_state.bearing_deg = math.degrees(imu.compass)
    dat = messaging.new_message('sensorEvents', 2)
    dat.sensorEvents[0].sensor = 4
    dat.sensorEvents[0].type = 0x10
    dat.sensorEvents[0].init('acceleration')
    dat.sensorEvents[0].acceleration.v = [
        imu.accelerometer.x, imu.accelerometer.y, imu.accelerometer.z]
    # copied these numbers from locationd
    dat.sensorEvents[1].sensor = 5
    dat.sensorEvents[1].type = 0x10
    dat.sensorEvents[1].init('gyroUncalibrated')
    dat.sensorEvents[1].gyroUncalibrated.v = [
        imu.gyroscope.x, imu.gyroscope.y, imu.gyroscope.z]
    pm.send('sensorEvents', dat)


def panda_state_function(vs: VehicleState, exit_event: threading.Event):
    pm = messaging.PubMaster(['pandaStates'])
    while not exit_event.is_set():
        dat = messaging.new_message('pandaStates', 1)
        dat.valid = True
        dat.pandaStates[0] = {
            'ignitionLine': vs.ignition,
            'pandaType': "blackPanda",
            'controlsAllowed': True,
            'safetyModel': 'hondaNidec'
        }
        pm.send('pandaStates', dat)
        time.sleep(0.5)


def peripheral_state_function(exit_event: threading.Event):
    pm = messaging.PubMaster(['peripheralState'])
    while not exit_event.is_set():
        dat = messaging.new_message('peripheralState')
        dat.valid = True
        # fake peripheral state data
        dat.peripheralState = {
            'pandaType': log.PandaState.PandaType.blackPanda,
            'voltage': 12000,
            'current': 5678,
            'fanSpeedRpm': 1000
        }
        pm.send('peripheralState', dat)
        time.sleep(0.5)


def gps_callback(gps, vehicle_state):
    dat = messaging.new_message('gpsLocationExternal')

    # transform vel from carla to NED
    # north is -Y in CARLA
    velNED = [
        -vehicle_state.vel.y,  # north/south component of NED is negative when moving south
        vehicle_state.vel.x,  # positive when moving east, which is x in carla
        vehicle_state.vel.z,
    ]

    dat.gpsLocationExternal = {
        "timestamp": int(time.time() * 1000),
        "flags": 1,  # valid fix
        "accuracy": 1.0,
        "verticalAccuracy": 1.0,
        "speedAccuracy": 0.1,
        "bearingAccuracyDeg": 0.1,
        "vNED": velNED,
        "bearingDeg": vehicle_state.bearing_deg,
        "latitude": gps.latitude,
        "longitude": gps.longitude,
        "altitude": gps.altitude,
        "speed": vehicle_state.speed,
        "source": log.GpsLocationData.SensorSource.ublox,
    }

    pm.send('gpsLocationExternal', dat)


def fake_driver_monitoring(exit_event: threading.Event):
    pm = messaging.PubMaster(['driverState', 'driverMonitoringState'])
    while not exit_event.is_set():
        # dmonitoringmodeld output
        dat = messaging.new_message('driverState')
        dat.driverState.faceProb = 1.0
        pm.send('driverState', dat)

        # dmonitoringd output
        dat = messaging.new_message('driverMonitoringState')
        dat.driverMonitoringState = {
            "faceDetected": True,
            "isDistracted": False,
            "awarenessStatus": 1.,
        }
        pm.send('driverMonitoringState', dat)

        time.sleep(DT_DMON)


def can_function_runner(vs: VehicleState, exit_event: threading.Event):
    i = 0
    while not exit_event.is_set():
        can_function(pm, vs.speed, vs.angle, i,
                     vs.cruise_button, vs.is_engaged)
        time.sleep(0.01)
        i += 1


def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2]:
            bps = [x for x in bps if int(
                x.get_attribute('generation')) == int_generation]
            return bps
        else:
            logger.warn(
                {"Event": "Actor Generation is not valid. No actor will be spawned."})
            return []
    except:
        logger.warn(
            {"Event": "Actor Generation is not valid. No actor will be spawned."})
        return []


def generate_traffic(client, world, traffic_manager, num_random_vehicles, num_walkers):
    spawn_points = world.get_map().get_spawn_points()
    random_vehicles_list = []
    # spawn other vehicles:
    if num_random_vehicles > 0:
        batch = []

        blueprints = get_actor_blueprints(world, 'vehicle.*', 'All')
        blueprints = [x for x in blueprints if int(
            x.get_attribute('number_of_wheels')) == 4]
        blueprints = [x for x in blueprints if not x.id.endswith('microlino')]
        blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
        blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
        blueprints = [x for x in blueprints if not x.id.endswith('t2')]
        blueprints = [x for x in blueprints if not x.id.endswith('sprinter')]
        blueprints = [x for x in blueprints if not x.id.endswith('firetruck')]
        blueprints = [x for x in blueprints if not x.id.endswith('ambulance')]
        blueprints = sorted(blueprints, key=lambda bp: bp.id)
        for n, transform in enumerate(spawn_points):
            if n >= num_random_vehicles:
                break
            if n == config['spawn_point']:
                continue
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(
                    blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(
                    blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            blueprint.set_attribute('role_name', 'autopilot')

            # spawn the cars and set their autopilot and light state all together
            batch.append(carla.command.SpawnActor(blueprint, transform)
                         .then(carla.command.SetAutopilot(carla.command.FutureActor, True, traffic_manager.get_port())))
        for response in client.apply_batch_sync(batch, True):
            if not response.error:
                random_vehicles_list.append(response.actor_id)

    world.set_pedestrians_seed(0)
    random.seed(0)
    walkers_list = []
    all_id = []

    if num_walkers > 0:
        blueprintsWalkers = get_actor_blueprints(
            world, 'walker.pedestrian.*', 'All')
        w_spawn_points = []
        for i in range(num_walkers):
            w_spawn_point = carla.Transform()
            loc = world.get_random_location_from_navigation()
            if (loc != None):
                w_spawn_point.location = loc
                w_spawn_points.append(w_spawn_point)
        # 2. we spawn the walker object
        w_batch = []
        walker_speed = []
        for w_spawn_point in w_spawn_points:
            walker_bp = random.choice(blueprintsWalkers)
            # set as not invincible
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            # set the max speed
            if walker_bp.has_attribute('speed'):
                walker_speed.append(walker_bp.get_attribute(
                    'speed').recommended_values[1])
            else:
                walker_speed.append(0.0)
            w_batch.append(carla.command.SpawnActor(walker_bp, w_spawn_point))
        results = client.apply_batch_sync(w_batch, True)
        walker_speed2 = []
        for i in range(len(results)):
            if results[i].error:
                continue
            walkers_list.append({"id": results[i].actor_id})
            walker_speed2.append(walker_speed[i])
        walker_speed = walker_speed2
        # 3. we spawn the walker controller
        w_batch = []
        walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(walkers_list)):
            w_batch.append(carla.command.SpawnActor(
                walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
        results = client.apply_batch_sync(w_batch, True)

        for i in range(len(results)):
            if results[i].error:

                continue
            walkers_list[i]["con"] = results[i].actor_id
        # 4. we put together the walkers and controllers id to get the objects from their id
        for i in range(len(walkers_list)):
            all_id.append(walkers_list[i]["con"])
            all_id.append(walkers_list[i]["id"])
        all_actors = world.get_actors(all_id)
        world.set_pedestrians_cross_factor(0.0)
        for i in range(0, len(all_id), 2):
            # start walker
            all_actors[i].start()
            # set walk to random point
            all_actors[i].go_to_location(
                world.get_random_location_from_navigation())
            # max speed
            all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))

    return random_vehicles_list, all_id


def bridge(q, itr, output_path):

    global config
    # setup CARLA
    client = carla.Client("127.0.0.1", 2000)

    client.set_timeout(10.0)

    world = client.load_world(config['town'])
    sim_time = round(world.get_snapshot().timestamp.elapsed_seconds, 3)
    logger.info({"SimRound": itr+1, "SimTime": sim_time,
                "Event": "Connected to Carla"})
    traffic_manager = client.get_trafficmanager(8000)

    traffic_manager.set_global_distance_to_leading_vehicle(2.5)
    traffic_manager.set_synchronous_mode(True)

    settings = world.get_settings()
    settings.synchronous_mode = True  # Enables synchronous mode
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    world.set_weather(carla.WeatherParameters.ClearSunset)

    if config['low_quality']:
        world.unload_map_layer(carla.MapLayer.Foliage)
        world.unload_map_layer(carla.MapLayer.Buildings)
        world.unload_map_layer(carla.MapLayer.ParkedVehicles)
        world.unload_map_layer(carla.MapLayer.Props)
        world.unload_map_layer(carla.MapLayer.StreetLights)
        world.unload_map_layer(carla.MapLayer.Particles)
    logger.info({"SimRound": itr+1, "SimTime": sim_time,
                "Event": "Applied world settings"})
    blueprint_library = world.get_blueprint_library()

    world_map = world.get_map()
    spawn_points = world_map.get_spawn_points()
    num_spawn_points = len(spawn_points)
    vehicle_bp = blueprint_library.filter('vehicle.tesla.*')[1]

    assert num_spawn_points > config['spawn_point'], \
        f'''No spawn point {config['spawn_point']}, try a value between 0 and
    {num_spawn_points} for this town.'''

    num_random_vehicles = config['n_rnd_vehicles']
    num_walkers = config['n_walkers']
    assert num_random_vehicles < num_spawn_points, f"requested {config['n_rnd_vehicles']}  vehicles, but could only find {num_spawn_points} spawn points"

    spawn_point = spawn_points[config['spawn_point']]
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    logger.info(
        {"SimRound": itr+1, "SimTime": sim_time, "Event": "Spawned a vehicle for openpilot"})
    max_steer_angle = vehicle.get_physics_control().wheels[0].max_steer_angle

    # make tires less slippery
    # wheel_control = carla.WheelPhysicsControl(tire_friction=5)
    physics_control = vehicle.get_physics_control()
    physics_control.mass = 2326
    # physics_control.wheels = [wheel_control]*4
    physics_control.torque_curve = [[20.0, 500.0], [5000.0, 500.0]]
    physics_control.gear_switch_time = 0.0
    vehicle.apply_physics_control(physics_control)

    transform = carla.Transform(carla.Location(x=0.8, z=1.13))

    def create_camera(fov, callback, config):

        blueprint = blueprint_library.find('sensor.camera.rgb')

        blueprint.set_attribute('image_size_x', str(W))
        blueprint.set_attribute('image_size_y', str(H))
        blueprint.set_attribute('fov', str(fov))
        blueprint.set_attribute('sensor_tick', '0.05')
        camera = world.spawn_actor(blueprint, transform, attach_to=vehicle)
        camera.listen(callback)
        return camera

    def create_faulty_camera(fov, callback, config):
        blueprint = blueprint_library.find('sensor.camera.frgb')
        if config['fault']['type'] == 'gaussian':
            blueprint.set_attribute('fault_type', 'Gaussian')
            seed = config['fault']['seed']
            minDist = config['fault']['min']
            maxDist = config['fault']['max']
            blueprint.set_attribute('faulty_parameters',
                                    f'seed:{seed}\ndistMin:{minDist}\ndistMax:{maxDist}')
        elif config['fault']['type'] == 'bac':
            alpha = config['fault']['alpha']
            beta = config['fault']['beta']
            blueprint.set_attribute('fault_type', 'BrightnessAndContrast')
            blueprint.set_attribute(
                'faulty_parameters', f'alpha:{alpha}\nbeta:{beta}')
        blueprint.set_attribute('image_size_x', str(W))
        blueprint.set_attribute('image_size_y', str(H))
        blueprint.set_attribute('fov', str(fov))
        blueprint.set_attribute('sensor_tick', '0.05')
        camera = world.spawn_actor(blueprint, transform, attach_to=vehicle)
        camera.listen(callback)
        return camera

    vehicle_state = VehicleState()
    camerad = Camerad(world, config['g_run'], vehicle_state, config['fault'])
    road_camera = create_camera(
        fov=40, callback=camerad.cam_callback_road, config=config)

    # fov bigger than 163 shows unwanted artifacts
    road_wide_camera = create_camera(
        fov=163, callback=camerad.cam_callback_wide_road, config=config)

    cameras = [road_camera, road_wide_camera]
    if not config['g_run']:
        road_faulty_camera = create_faulty_camera(
            fov=40, callback=camerad.cam_callback_road_faulty, config=config)
        cameras.append(road_faulty_camera)
    logger.info({"SimRound": itr+1, "SimTime": sim_time,
                "Event": "Created cameras"})

    # reenable IMU
    imu_bp = blueprint_library.find('sensor.other.imu')
    imu = world.spawn_actor(imu_bp, transform, attach_to=vehicle)
    imu.listen(lambda imu: imu_callback(imu, vehicle_state))

    gps_bp = blueprint_library.find('sensor.other.gnss')
    gps = world.spawn_actor(gps_bp, transform, attach_to=vehicle)
    gps.listen(lambda gps: gps_callback(gps, vehicle_state))

    # launch fake car threads
    threads = []
    exit_event = threading.Event()
    threads.append(threading.Thread(target=panda_state_function,
                   args=(vehicle_state, exit_event,)))
    threads.append(threading.Thread(
        target=peripheral_state_function, args=(exit_event,)))
    threads.append(threading.Thread(
        target=fake_driver_monitoring, args=(exit_event,)))
    threads.append(threading.Thread(target=can_function_runner,
                   args=(vehicle_state, exit_event,)))
    for t in threads:
        t.start()

    # can loop
    rk = Ratekeeper(100, print_delay_threshold=0.05)

    # init
    throttle_ease_out_counter = REPEAT_COUNTER
    brake_ease_out_counter = REPEAT_COUNTER
    steer_ease_out_counter = REPEAT_COUNTER

    vc = carla.VehicleControl(throttle=0, steer=0, brake=0, reverse=False)

    is_openpilot_engaged = False
    throttle_out = steer_out = brake_out = 0
    throttle_op = steer_op = brake_op = 0
    throttle_manual = steer_manual = brake_manual = 0

    old_steer = old_brake = old_throttle = 0
    throttle_manual_multiplier = 0.7  # keyboard signal is always 1
    brake_manual_multiplier = 0.7  # keyboard signal is always 1
    steer_manual_multiplier = 45 * STEER_RATIO  # keyboard signal is always 1

    lead_vehicle = None
    safety_violated = False
    lead_vehicle_stopped = False
    output = pd.DataFrame(columns=['datetime', 'sim_round', 'sim_time', 'rel_distance', 'fault_activated', 'fault_min', 'fault_max', 'safety_violated',
                          'brake_delay', 'op_speed', 'op_steer', 'op_throttle', 'op_brake'])

    logger.info(
        {"SimRound": itr+1, "SimTime": sim_time, "Event": "Started interacting with openpilot"})
    while True:
        # 1. Read the throttle, steer and brake from op or manual controls
        # 2. Set instructions in Carla
        # 3. Send current carstate to op via can

        cruise_button = 0
        throttle_out = steer_out = brake_out = 0.0
        throttle_op = steer_op = brake_op = 0
        throttle_manual = steer_manual = brake_manual = 0.0

        # --------------Step 1-------------------------------
        if not q.empty():
            message = q.get()

            m = message.split('_')
            if m[0] == "steer":
                steer_manual = float(m[1])
                is_openpilot_engaged = False
            elif m[0] == "throttle":
                throttle_manual = float(m[1])
                is_openpilot_engaged = False
            elif m[0] == "brake":
                brake_manual = float(m[1])
                is_openpilot_engaged = False
            elif m[0] == "reverse":
                cruise_button = CruiseButtons.CANCEL
                is_openpilot_engaged = False
            elif m[0] == "cruise":
                if m[1] == "down":
                    cruise_button = CruiseButtons.DECEL_SET
                    is_openpilot_engaged = True
                elif m[1] == "up":
                    cruise_button = CruiseButtons.RES_ACCEL
                    is_openpilot_engaged = True
                elif m[1] == "cancel":
                    cruise_button = CruiseButtons.CANCEL
                    is_openpilot_engaged = False
            elif m[0] == "ignition":
                vehicle_state.ignition = not vehicle_state.ignition
            elif m[0] == "quit":

                break

            throttle_out = throttle_manual * throttle_manual_multiplier
            steer_out = steer_manual * steer_manual_multiplier
            brake_out = brake_manual * brake_manual_multiplier

            old_steer = steer_out
            old_throttle = throttle_out
            old_brake = brake_out

        if is_openpilot_engaged:
            sm.update(0)

            # TODO gas and brake is deprecated
            throttle_op = clip(
                sm['carControl'].actuators.accel / 1.6, 0.0, 1.0)
            brake_op = clip(-sm['carControl'].actuators.accel / 4.0, 0.0, 1.0)
            steer_op = sm['carControl'].actuators.steeringAngleDeg

            throttle_out = throttle_op
            steer_out = steer_op
            brake_out = brake_op

            steer_out = steer_rate_limit(old_steer, steer_out)
            old_steer = steer_out

        else:
            if throttle_out == 0 and old_throttle > 0:
                if throttle_ease_out_counter > 0:
                    throttle_out = old_throttle
                    throttle_ease_out_counter += -1
                else:
                    throttle_ease_out_counter = REPEAT_COUNTER
                    old_throttle = 0

            if brake_out == 0 and old_brake > 0:
                if brake_ease_out_counter > 0:
                    brake_out = old_brake
                    brake_ease_out_counter += -1
                else:
                    brake_ease_out_counter = REPEAT_COUNTER
                    old_brake = 0

            if steer_out == 0 and old_steer != 0:
                if steer_ease_out_counter > 0:
                    steer_out = old_steer
                    steer_ease_out_counter += -1
                else:
                    steer_ease_out_counter = REPEAT_COUNTER
                    old_steer = 0

        # --------------Step 2-------------------------------
        steer_carla = steer_out / (max_steer_angle * STEER_RATIO * -1)

        steer_carla = np.clip(steer_carla, -1, 1)
        steer_out = steer_carla * (max_steer_angle * STEER_RATIO * -1)
        old_steer = steer_carla * (max_steer_angle * STEER_RATIO * -1)

        vc.throttle = throttle_out / 0.6
        vc.steer = steer_carla
        vc.brake = brake_out
        vehicle.apply_control(vc)

        # --------------Step 3-------------------------------
        vel = vehicle.get_velocity()
        speed = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)  # in m/s
        vehicle_state.speed = speed
        vehicle_state.vel = vel
        vehicle_state.angle = steer_out
        vehicle_state.cruise_button = cruise_button
        vehicle_state.is_engaged = is_openpilot_engaged
        sim_time = round(world.get_snapshot().timestamp.elapsed_seconds, 3)
        if not lead_vehicle and speed > 2.:
            logger.info(
                {"SimRound": itr+1, "SimTime": sim_time, "Event": "Openpilot started driving"})
            vehicle_state.start_delay = sim_time
            config['fault']['activation_time'] += sim_time
            config['fault']['deactivation_time'] += sim_time
            config['sim_timeout'] += sim_time
            random_vehicles_list, all_id = generate_traffic(
                client, world, traffic_manager, num_random_vehicles, num_walkers)
            logger.info(
                {"SimRound": itr+1, "SimTime": sim_time, "Event": "Generated traffic"})
            # spawn lead vehicle
            lead_vehicle_bp = world.get_blueprint_library().filter(
                'vehicle.audi.*')[1]
            lead_vehicle_bp.set_attribute('role_name', 'hero')
            if lead_vehicle_bp.has_attribute('color'):
                color = random.choice(
                    lead_vehicle_bp.get_attribute('color').recommended_values)
            lead_vehicle_bp.set_attribute('color', color)
            vehicle_location = spawn_point.location
            # vehicle_direction = vehicle.get_transform().get_forward_vector()
            # vehicle_rotation = vehicle.get_transform().rotation

            # lead_vehicle_location = carla.Location(
            #    vehicle_location.x+10, vehicle_location.y, vehicle_location.z+2)
            vehicle_waypoint = world.get_map().get_waypoint(vehicle_location)
            lead_vehicle_waypoint = vehicle_waypoint.next(
                config['scenario']['init_distance'])[0]
            lead_vehicle_location = lead_vehicle_waypoint.transform.location + \
                carla.Location(0, 0, 2)
            lead_vehicle_rotation = lead_vehicle_waypoint.transform.rotation

            response = client.apply_batch_sync([carla.command.SpawnActor(lead_vehicle_bp, carla.Transform(
                lead_vehicle_location, lead_vehicle_rotation)).then(carla.command.SetAutopilot(carla.command.FutureActor, True, traffic_manager.get_port()))], True)
            lead_vehicle = world.get_actors().find(response[0].actor_id)
            logger.info(
                {"SimRound": itr+1, "SimTime": sim_time, "Event": "Spawned the lead vehicle"})
            # lead_vehicle = world.try_spawn_actor(lead_vehicle_bp, carla.Transform(
            #    lead_vehicle_location, lead_vehicle_rotation))
            # lead_vehicle.set_autopilot(True, traffic_manager.get_port())
            # print(spawn_point.location, lead_vehicle_location)
            # for v in random_vehicles_list:
            #    print(world.get_actors().find(v).get_transform().location.x)
            # sys.exit()
            try:
                physics_control = lead_vehicle.get_physics_control()
                physics_control.use_sweep_wheel_collision = True
                lead_vehicle.apply_physics_control(physics_control)
            except Exception:
                pass
            # agent = BehaviorAgent(lead_vehicle, behavior='cautious')
            agent = BasicAgent(lead_vehicle)
            for sp in spawn_points:
                if sp == spawn_point:
                    continue
                distance = spawn_point.location.distance(sp.location)
                if distance > 100.0:
                    agent.set_destination(sp.location)
                    break

        if rk.frame % 5 == 0:
            rel_distance = None
            if lead_vehicle:
                control = agent.run_step()
                control.manual_gear_shift = False
                brake_delay = config['scenario']['brake_delay']
                if sim_time >= brake_delay + vehicle_state.start_delay:

                    lead_vehicle.set_autopilot(
                        False, traffic_manager.get_port())
                    control.brake = 0.8
                    if not lead_vehicle_stopped:  # just to avoid too many log events

                        logger.info(
                            {"SimRound": itr+1, "SimTime": sim_time, "Event": "Stopped the lead vehicle"})
                    lead_vehicle_stopped = True
                lead_vehicle.apply_control(control)
                for param, value in config['requirements'].items():
                    if param == 'relative_distance':
                        rel_distance = round(
                            lead_vehicle.get_location().distance(vehicle.get_location()), 3)
                        if rel_distance < value:
                            safety_violated = True
                            logger.info(
                                {"SimRound": itr+1, "SimTime": sim_time, "Event": "Safety violated"})
                if safety_violated:

                    while not q.empty():
                        q.get()
                    q.put('quit')

            if rk.frame % PRINT_DECIMATION == 0:
                if lead_vehicle and sim_time >= config['sim_timeout']:
                    logger.info(
                        {"SimRound": itr+1, "SimTime": sim_time, "Event": "Simulation timeout"})
                    while not q.empty():
                        q.get()
                    q.put('quit')
                if lead_vehicle:
                    f_activated = sim_time >= config[
                        'fault']['activation_time'] and sim_time < config[
                        'fault']['deactivation_time']
                    data = dict(datetime=datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S'),
                                sim_round=[itr+1],
                                sim_time=[round(sim_time, 3)],
                                rel_distance=[rel_distance],
                                fault_activated=[f_activated],
                                fault_min=config['fault']['min'],
                                fault_max=config['fault']['max'],
                                safety_violated=[safety_violated],
                                brake_delay=[
                                    config['scenario']['brake_delay']],
                                op_speed=[round(speed*2.2369, 3)],
                                # op_engaged=is_openpilot_engaged,
                                op_steer=[round(vc.steer, 3)],
                                op_throttle=[round(vc.throttle, 3)],
                                op_brake=[round(vc.brake, 3)])
                    output = pd.concat(
                        [output, pd.DataFrame.from_dict(data)], ignore_index=True)

            world.tick()
        rk.keep_time()

    output.to_csv(output_path, mode='a',
                  header=not os.path.exists(output_path), index=False)

    # Clean up resources in the opposite order they were created.

    exit_event.set()
    for t in reversed(threads):
        t.join()
    gps.destroy()
    imu.destroy()
    for c in cameras:
        c.destroy()

    if lead_vehicle:
        lead_vehicle.destroy()
        if num_random_vehicles > 0:
            client.apply_batch([carla.command.DestroyActor(x)
                                for x in random_vehicles_list])
        if num_walkers > 0:
            client.apply_batch([carla.command.DestroyActor(x)
                                for x in all_id])
    vehicle.destroy()


def bridge_keep_alive(q: Any, itr: int, output_path: str):
    while 1:
        try:

            bridge(q, itr, output_path)

            break
        except RuntimeError as e:
            logger.error(
                {"Event": "Restarting bridge, Error: "+str(e)})


def create_arg_parser():
    parser = argparse.ArgumentParser(
        description='CarFI campaign manager.', formatter_class=RawTextHelpFormatter)
    group = parser.add_mutually_exclusive_group()
    group.add_argument('-g', '--golden_run', action='store_true',
                       help='run the driving scenario without fault injection')
    group.add_argument('-f', '--fault_params', nargs='+',
                       help='specify the fault to be injected:\n'
                       '\tGaussian noise: G seed minDist maxDist\n'
                       '\tBrightness and Contrast: BAC alpha beta\n')
    parser.add_argument('-l', '--low_quality', action='store_true',
                        help='run CARLA in the low quality mode to increase performance')
    parser.add_argument('-t', '--town', type=str,
                        help='select the map to drive in')
    parser.add_argument('-w', '--weather', type=str,
                        help='specify the whether condition')
    parser.add_argument('-s',
                        '--spawn_point', dest='num_selected_spawn_point', type=int, help='number of the spawn point to start at)')
    parser.add_argument('-n',
                        '--sim_rounds', dest='num_runs', type=int, help='number of simulation rounds)')
    parser.add_argument('-u', '--update_config', action='store_true',
                        help='update the configuration file with the given settings')
    return parser


def run_openpilot():
    subprocess.run('./launch_openpilot.sh',
                   stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)


def run_sim(config, output_path):

    for i in range(config['sim_rounds']):
        p_op = Process(target=run_openpilot, daemon=False)
        p_op.start()
        # make sure params are in a good state
        for key in config['fault']:
            if key in rand_vars.keys():
                config['fault'][key] = rand_vars[key].pop()
        for key in config['scenario']:
            if key in rand_vars.keys():
                config['scenario'][key] = rand_vars[key].pop()

        q: Any = Queue()
        p = Process(target=bridge_keep_alive, args=(
            q, i, output_path), daemon=False)
        p.start()

        cmd_list = []
        for cmd, cnt in config['scenario']['cmds']:
            cmd_list += list(cmd*cnt)
        cmd_list.append('q')

        for c in cmd_list:
            if not p.is_alive():
                break
            if c == '1':
                q.put("cruise_up")
            elif c == '2':
                q.put("cruise_down")
            elif c == '3':
                q.put("cruise_cancel")
            elif c == 'w':
                q.put("throttle_%f" % 1.0)
            elif c == 'a':
                q.put("steer_%f" % 0.15)
            elif c == 's':
                q.put("brake_%f" % 1.0)
            elif c == 'd':
                q.put("steer_%f" % -0.15)
            elif c == 'i':
                q.put("ignition")
            elif c == 'q':
                q.put("quit")
                break
            time.sleep(config['scenario']['cmd_delay'])

        p.join()
        if p.is_alive():
            p.terminate()
        p.close()


def analyze_measures(config, output_path):
    measures = config['measures']
    analyzer = analysis.Analyzer(output_path)
    for msr_type, msr_params in measures.items():
        if msr_type == 'corr':
            analyzer.corr(msr_params)
        else:
            for msr in msr_params:
                getattr(analyzer, msr)(config)


if __name__ == "__main__":
    output_path = './out/output_'+datetime.utcnow().strftime('%Y-%m-%d_%H_%M')+'.csv'
    set_params_enabled()

    msg = messaging.new_message('liveCalibration')
    msg.liveCalibration.validBlocks = 20
    msg.liveCalibration.rpyCalib = [0.0, 0.0, 0.0]
    Params().put("CalibrationParams", msg.to_bytes())
    parser = create_arg_parser()
    args = parser.parse_args()
    with open('carfi.config', 'r') as config_file:
        config = json.load(config_file)

    if args.fault_params:
        f_sensor = args.fault_params[0]
        if f_sensor == 'gaussian':
            if len(args.fault_params) != 4:
                parser.error(
                    'the Gaussian fault model needs 3 parameters: (seed, min, max)')
            config['fault']['type'] = f_sensor
            config['fault']['seed'] = int(args.fault_params[1])
            config['fault']['min'] = float(args.fault_params[2])
            config['fault']['max'] = float(args.fault_params[3])
        elif f_sensor == 'bac':
            if len(args.fault_params) != 3:
                parser.error(
                    'the BrightnessAndContrast fault model needs 2 parameters: (alpha, beta)')
            config['fault']['type'] = f_sensor
            config['fault']['alpha'] = float(args.fault_params[0])
            config['fault']['beta'] = float(args.ffault_params[1])

    config['low_quality'] = args.low_quality
    config['g_run'] = args.golden_run
    if args.town:
        config['town'] = args.town
    if args.weather:
        config['weather'] = args.weather
    if args.num_selected_spawn_point:
        config['spawn_point'] = args.num_selected_spawn_point
    if args.num_runs:
        config['sim_rounds'] = args.num_runs
    if args.update_config:
        with open('carfi.config', 'w') as config_file:
            json.dump(config, config_file, indent=4)
    rand_vars = dict()
    for key, val in config['fault'].items():
        if isinstance(val, dict):
            if val['dist'] == 'uniform':
                min = val['min']
                max = val['max']
                rand_vars[key] = [round(
                    random.uniform(min, max), 3) for i in range(config['sim_rounds'])]

    for key, val in config['scenario'].items():
        if isinstance(val, dict):
            if val['dist'] == 'uniform':
                min = val['min']
                max = val['max']
                rand_vars[key] = [round(
                    random.uniform(min, max), 3) for i in range(config['sim_rounds'])]

    run_sim(config, output_path)
    analyze_measures(config, output_path)
