#!/usr/bin/env python3
import util
import fa_lib
from numpy import random
import math
import threading
import time
import os
import sys
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
from common.realtime import DT_DMON, Ratekeeper
from selfdrive.car.honda.values import CruiseButtons
from agents.navigation.basic_agent import BasicAgent
from datetime import datetime


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
        self.has_crashed = False


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
    def __init__(self, world, vehicle_state, image_q=None):
        self.world = world
        self.config = util.get_config()
        self.image_q = image_q

        self.vehicle_state = vehicle_state

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

        noise_func = None
        if not self.config['golden run'] and self.vehicle_state.start_delay > 0:
            sim_time = round(
                self.world.get_snapshot().timestamp.elapsed_seconds, 3)
            if sim_time >= util.get_config_val(self.config['fault activation time']) and sim_time < util.get_config_val(self.config['fault deactivation time']):
                if self.config['fault implementation']['value'] == 'server':
                    return
                noise_func = self.config['fault type']['value']

        self._cam_callback(image, self.frame_road_id, 'roadCameraState',
                           VisionStreamType.VISION_STREAM_RGB_ROAD, VisionStreamType.VISION_STREAM_ROAD, noise_func, self.image_q)
        self.frame_road_id += 1

    def cam_callback_road_faulty(self, image):
        if not self.config['golden run']:
            if self.vehicle_state.start_delay == 0.0:
                return

            sim_time = round(
                self.world.get_snapshot().timestamp.elapsed_seconds, 3)

            if sim_time >= util.get_config_val(self.config['fault deactivation time'], False) or sim_time < util.get_config_val(self.config['fault activation_time'], False):
                return

        self._cam_callback(image, self.frame_road_id, 'roadCameraState',
                           VisionStreamType.VISION_STREAM_RGB_ROAD, VisionStreamType.VISION_STREAM_ROAD, image_q=self.image_q)
        self.frame_road_id += 1

    def cam_callback_wide_road(self, image):
        self._cam_callback(image, self.frame_wide_id, 'wideRoadCameraState',
                           VisionStreamType.VISION_STREAM_RGB_WIDE_ROAD, VisionStreamType.VISION_STREAM_WIDE_ROAD)
        self.frame_wide_id += 1

    def _cam_callback(self, image, frame_id, pub_type, rgb_type, yuv_type, noise_func=None, image_q=None):
        img = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        img = np.reshape(img, (H, W, 4))
        img = img[:, :, [0, 1, 2]].copy()
        if noise_func:
            img = getattr(fa_lib, noise_func)(img, self.config['fault params'])

        if image_q:

            image_q.put(img)
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


def cs_callback(cs, vehicle_state):

    vehicle_state.has_crashed = True


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
    logger = util.get_logger()
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
                {"SimRound": -1, "SimTime": -1, "Event": "Actor Generation is not valid. No actor will be spawned."})
            return []
    except:
        logger.warn(
            {"SimRound": -1, "SimTime": -1, "Event": "Actor Generation is not valid. No actor will be spawned."})
        return []


def generate_traffic(client, world, traffic_manager, num_random_vehicles, num_walkers, spawn_point):
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
            if n == spawn_point:
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


def setup_carla(config):

    client = carla.Client("127.0.0.1", 2000)

    client.set_timeout(10.0)

    world = client.load_world(config['town']['value'])

    traffic_manager = client.get_trafficmanager(8000)

    traffic_manager.set_global_distance_to_leading_vehicle(2.5)
    traffic_manager.set_synchronous_mode(True)

    settings = world.get_settings()
    settings.synchronous_mode = True  # Enables synchronous mode
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    world.set_weather(carla.WeatherParameters.ClearSunset)

    world.unload_map_layer(carla.MapLayer.Foliage)
    world.unload_map_layer(carla.MapLayer.Buildings)
    world.unload_map_layer(carla.MapLayer.ParkedVehicles)
    world.unload_map_layer(carla.MapLayer.Props)
    world.unload_map_layer(carla.MapLayer.StreetLights)
    world.unload_map_layer(carla.MapLayer.Particles)
    return client, world, traffic_manager


def spawn_op_vehicle(world, config):
    blueprint_library = world.get_blueprint_library()
    world_map = world.get_map()
    spawn_points = world_map.get_spawn_points()

    spawn_point = spawn_points[config['spawn point']]

    vehicle_bp = blueprint_library.filter('vehicle.tesla.*')[1]
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)

    # make tires less slippery
    # wheel_control = carla.WheelPhysicsControl(tire_friction=5)
    physics_control = vehicle.get_physics_control()
    physics_control.mass = 2326
    # physics_control.wheels = [wheel_control]*4
    physics_control.torque_curve = [[20.0, 500.0], [5000.0, 500.0]]
    physics_control.gear_switch_time = 0.0
    vehicle.apply_physics_control(physics_control)
    return vehicle, spawn_point


def set_destination(world, agent, op_spawn_point):
    for sp in world.get_map().get_spawn_points():
        if sp == op_spawn_point:
            continue
        distance = op_spawn_point.location.distance(sp.location)
        if distance > 100.0:
            agent.set_destination(sp.location)
            break


def create_camera(world, vehicle, transform, fov, callback):
    blueprint_library = world.get_blueprint_library()
    blueprint = blueprint_library.find('sensor.camera.rgb')

    blueprint.set_attribute('image_size_x', str(W))
    blueprint.set_attribute('image_size_y', str(H))
    blueprint.set_attribute('fov', str(fov))
    blueprint.set_attribute('sensor_tick', '0.05')
    transform = carla.Transform(carla.Location(x=0.8, z=1.13))
    camera = world.spawn_actor(blueprint, transform, attach_to=vehicle)
    camera.listen(callback)
    return camera


def create_faulty_camera(world, vehicle, transform, fov, callback):
    config = util.get_config()
    blueprint_library = world.get_blueprint_library()
    blueprint = blueprint_library.find('sensor.camera.frgb')
    if config['fault type'] == 'gaussian':
        blueprint.set_attribute('fault_type', 'Gaussian')
        seed = util.get_config_val(config['fault params']['seed'])
        minDist = util.get_config_val(config['fault params']['min'])
        maxDist = util.get_config_val(config['fault params']['max'])
        blueprint.set_attribute('faulty_parameters',
                                f'seed:{seed}\ndistMin:{minDist}\ndistMax:{maxDist}')
    elif config['fault']['type'] == 'bac':
        alpha = util.get_config_val(config['fault params']['alpha'])
        beta = util.get_config_val(config['fault params']['beta'])
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


def create_cameras(world, vehicle, vehicle_state, transform, image_q):
    config = util.get_config()
    camerad = Camerad(world, vehicle_state, image_q)
    road_camera = create_camera(world, vehicle, transform,
                                fov=40, callback=camerad.cam_callback_road)

    # fov bigger than 163 shows unwanted artifacts
    road_wide_camera = create_camera(world, vehicle, transform,
                                     fov=163, callback=camerad.cam_callback_wide_road)

    cameras = [road_camera, road_wide_camera]
    if not config['golden run'] and config['fault implementation']['value'] == 'server':
        road_faulty_camera = create_faulty_camera(world, vehicle,
                                                  fov=40, callback=camerad.cam_callback_road_faulty)
        cameras.append(road_faulty_camera)
    return cameras


def create_imu(world, vehicle, vehicle_state, transform):
    blueprint_library = world.get_blueprint_library()
    imu_bp = blueprint_library.find('sensor.other.imu')
    imu = world.spawn_actor(imu_bp, transform, attach_to=vehicle)
    imu.listen(lambda imu: imu_callback(imu, vehicle_state))
    return imu


def create_gps(world, vehicle, vehicle_state, transform):
    blueprint_library = world.get_blueprint_library()
    gps_bp = blueprint_library.find('sensor.other.gnss')
    gps = world.spawn_actor(gps_bp, transform, attach_to=vehicle)
    gps.listen(lambda gps: gps_callback(gps, vehicle_state))
    return gps


def create_cs(world, vehicle, vehicle_state, transform):
    blueprint_library = world.get_blueprint_library()
    cs_bp = blueprint_library.find('sensor.other.collision')
    cs = world.spawn_actor(cs_bp, transform, attach_to=vehicle)
    cs.listen(lambda cs: cs_callback(cs, vehicle_state))
    return cs


class Scenario:
    def __init__(self, image_q=None):
        self.image_q = image_q
        self.client = None

    def sudden_brake(self, q, itr):
        config = util.get_config()
        logger = util.get_logger()
        if not self.client:
            self.client, self.world, self.traffic_manager = setup_carla(config)
        client = self.client
        world = self.world
        traffic_manager = self.traffic_manager

        sim_time = round(world.get_snapshot().timestamp.elapsed_seconds, 3)
        logger.info({"SimRound": itr+1, "SimTime": sim_time,
                    "Event": "Connected to Carla"})
        vehicle, spawn_point = spawn_op_vehicle(world, config)

        transform = carla.Transform(carla.Location(x=0.8, z=1.13))
        vehicle_state = VehicleState()
        cameras = create_cameras(world, vehicle, vehicle_state, transform, self.image_q)

        # reenable IMU
        imu = create_imu(world, vehicle, vehicle_state, transform)
        gps = create_gps(world, vehicle, vehicle_state, transform)
        cs = create_cs(world, vehicle, vehicle_state, transform)

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
        max_steer_angle = vehicle.get_physics_control().wheels[0].max_steer_angle

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

        op_started = False
        lead_vehicle = None

        lead_vehicle_stopped = False
        output = pd.DataFrame()
        num_random_vehicles = config['# random vehicles']
        num_walkers = config['# walkers']

        while True:
            # 1. Read the throttle, steer and brake from op or manual controls
            # 2. Set instructions in Carla
            # 3. Send current carstate to op via can
            safety_violated = False
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
            if not op_started and speed > 2.:
                op_started = True
                logger.info(
                    {"SimRound": itr+1, "SimTime": sim_time, "Event": "Openpilot started driving"})

                vehicle_state.start_delay = sim_time
                config['scenario time'] += sim_time

                util.init_config_val(config['fault activation time'], sim_time)

                util.init_config_val(config['fault deactivation time'], sim_time)
                util.init_config_val(config['scenario params']['brake delay'], sim_time)
                util.init_config_val(config['scenario params']['init distance'])
                util.init_config_val(config['scenario params']['safe distance'])
                for fp in config['fault params']:
                    util.init_config_val(config['fault params'][fp])
                config['scenario time'] += sim_time

                random_vehicles_list, all_id = generate_traffic(
                    client, world, traffic_manager, num_random_vehicles, num_walkers, config['spawn point'])

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
                    util.get_config_val(config['scenario params']['init distance']))[0]
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
                set_destination(world, agent, spawn_point)

            if rk.frame % 5 == 0:

                rel_distance = None
                if op_started:
                    control = agent.run_step()
                    control.manual_gear_shift = False
                    brake_delay = util.get_config_val(config['scenario params']['brake delay'])
                    if sim_time >= brake_delay + vehicle_state.start_delay:

                        lead_vehicle.set_autopilot(
                            False, traffic_manager.get_port())
                        control.brake = 0.8
                        if not lead_vehicle_stopped:  # just to avoid too many log events

                            logger.info(
                                {"SimRound": itr+1, "SimTime": sim_time, "Event": "Stopped the lead vehicle"})
                        lead_vehicle_stopped = True
                    lead_vehicle.apply_control(control)
                    safe_distance = util.get_config_val(config['scenario params']['safe distance'])

                    rel_distance = round(
                        lead_vehicle.get_location().distance(vehicle.get_location()), 3)
                    if not safety_violated and rel_distance < safe_distance:
                        safety_violated = True
                        logger.info(
                            {"SimRound": itr+1, "SimTime": sim_time, "Event": "Safety violated"})
                    else:
                        safety_violated = False
                    if vehicle_state.has_crashed:
                        logger.info(
                            {"SimRound": itr+1, "SimTime": sim_time, "Event": "Collision detected"})
                        while not q.empty():
                            q.get()
                        q.put('quit')

                    if sim_time >= config['scenario time']:
                        logger.info(
                            {"SimRound": itr+1, "SimTime": sim_time, "Event": "Scenario timeout"})
                        while not q.empty():
                            q.get()
                        q.put('quit')

                    fault_activation_time = util.get_config_val(
                        config['fault activation time'])
                    fault_deactivation_time = util.get_config_val(
                        config['fault deactivation time'])
                    fault_is_active = sim_time >= fault_activation_time and sim_time < fault_deactivation_time
                    data = {'datetime': datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S'),
                            'experiment': [itr+1],
                            'simulation time': [round(sim_time, 3)],
                            'op started': op_started,
                            'fault is active': fault_is_active,
                            'safety violated': safety_violated,
                            'crashed': vehicle_state.has_crashed,
                            'speed': [round(speed*2.2369, 3)],
                            'steer': [round(vc.steer, 3)],
                            'throttle': [round(vc.throttle, 3)],
                            'brake': [round(vc.brake, 3)]}
                    for fp in config['fault params']:
                        data[fp] = round(util.get_config_val(config['fault params'][fp]), 3)
                    for sp in config['scenario params']:
                        data[sp] = round(util.get_config_val(config['scenario params'][sp]), 3)
                    # todo: add requirments and analysis
                    output = pd.concat(
                        [output, pd.DataFrame.from_dict(data)], ignore_index=True)

                world.tick()
            rk.keep_time()
        output_path = util.output_path()
        output.to_csv(output_path, mode='a',
                      header=not os.path.exists(output_path), index=False)

        # Clean up resources in the opposite order they were created.

        exit_event.set()
        for t in reversed(threads):
            t.join()
        gps.destroy()
        imu.destroy()
        cs.destroy()
        for c in cameras:
            c.destroy()

        if op_started:
            lead_vehicle.destroy()
            if num_random_vehicles > 0:
                client.apply_batch([carla.command.DestroyActor(x)
                                    for x in random_vehicles_list])
            if num_walkers > 0:
                client.apply_batch([carla.command.DestroyActor(x)
                                    for x in all_id])
        vehicle.destroy()
