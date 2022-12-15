#!/usr/bin/env python3
import sim
import util
import analysis
import argparse
import subprocess
from argparse import RawTextHelpFormatter
import os
import signal
import time

from multiprocessing import Process, Queue
from typing import Any

import cereal.messaging as messaging

from common.params import Params
from selfdrive.test.helpers import set_params_enabled


from datetime import datetime


def run_sim_round(q, rnd, scenario):
    config = util.get_config()
    logger = util.get_logger()
    while 1:
        try:
            getattr(scenario, config['scenario name']['value'])(q, rnd)
            break
        except RuntimeError as e:
            logger.error(
                {"SimRound": -1, "SimTime": -1, "Event": "Restarting the simulation: "+str(e)})


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
    parser.add_argument('-t', '--town', type=str,
                        help='select the map to drive in')
    parser.add_argument('-w', '--weather', type=str,
                        help='specify the whether condition')
    parser.add_argument('-s',
                        '--spawn_point', dest='num_selected_spawn_point', type=int, help='number of the spawn point to start at)')
    parser.add_argument('-n',
                        '--sim_rounds', dest='num_runs', type=int, help='number of # experiments)')
    parser.add_argument('-u', '--update_config', action='store_true',
                        help='update the configuration file with the given settings')
    return parser


def run_openpilot():
    return subprocess.Popen('./launch_openpilot.sh',
                            stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)


def run_sim(image_q=None):
    config = util.get_config()
    scenario = sim.Scenario(image_q)

    for i in range(config['# experiments']):
        p_op = run_openpilot()
        q: Any = Queue()
        p = Process(target=run_sim_round, args=(
            q, i, scenario), daemon=False)
        p.start()

        cmd_list = []

        openpilot_commands = [["1", 100], ["2",  1]]  # too many cmds just to keep openpilot alive

        for cmd, cnt in openpilot_commands:
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
            time.sleep(2)

        p.join()
        if p.is_alive():
            p.terminate()
        p.close()
    if p_op:
        p_op.kill()


def analyze_measures(measures, data_path):

    for msr in measures:
        getattr(analysis, msr)(data_path)


def init_params():
    set_params_enabled()
    msg = messaging.new_message('liveCalibration')
    msg.liveCalibration.validBlocks = 20
    msg.liveCalibration.rpyCalib = [0.0, 0.0, 0.0]
    Params().put("CalibrationParams", msg.to_bytes())


if __name__ == "__main__":

    init_params()
    config = util.get_config()
    logger = util.get_logger()
    parser = create_arg_parser()
    args = parser.parse_args()

    if args.fault_params:
        f_sensor = args.fault_params[0]
        if f_sensor == 'gaussian':
            if len(args.fault_params) != 4:
                parser.error(
                    'the Gaussian fault model needs 3 parameters: (seed, min, max)')
            config['fault type'] = f_sensor
            config['fault params']['seed'] = int(args.fault_params[1])
            config['fault params']['min'] = float(args.fault_params[2])
            config['fault params']['max'] = float(args.fault_params[3])
        elif f_sensor == 'bac':
            if len(args.fault_params) != 3:
                parser.error(
                    'the BrightnessAndContrast fault model needs 2 parameters: (alpha, beta)')
            config['fault type'] = f_sensor
            config['fault params']['alpha'] = float(args.fault_params[0])
            config['fault params']['beta'] = float(args.ffault_params[1])

    config['golden run'] = args.golden_run
    if args.town:
        config['town']['value'] = args.town
    if args.weather:
        config['weather']['value'] = args.weather
    if args.num_selected_spawn_point:
        config['spawn point'] = args.num_selected_spawn_point
    if args.num_runs:
        config['# experiments'] = args.num_runs
    if args.update_config:
        util.save_config()

    run_sim()
    analyze_measures(config['measures']['chosen'], util.output_path())
