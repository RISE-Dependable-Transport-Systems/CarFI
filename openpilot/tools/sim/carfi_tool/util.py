import random
import json
from pythonjsonlogger import jsonlogger
from datetime import datetime
import logging
from inspect import getmembers, isfunction, getsource
import sim
import analysis
import fa_lib
import re
import os
import io
import zipfile
import time
import sys
datetime_format = '%Y-%m-%d_%H_%M'
startup_time = datetime.utcnow().strftime(datetime_format)
logger = None
config = None


def get_logger():
    global logger
    if logger is None:
        logger = init_logger()
    return logger


def get_config(auto_update=False):
    global config
    if config is None:
        config = load_config(auto_update)
    return config


def all_files(fdir, ftype, with_ext=True):

    files = [f for f in os.listdir(fdir) if f.endswith(ftype)]

    if not with_ext:
        files = [f.split('.')[0] for f in files]

    return files


def compress_files(fdir, files):

    fileobj = io.BytesIO()
    with zipfile.ZipFile(fileobj, 'w') as zip_file:
        for f in files:
            zip_info = zipfile.ZipInfo(f)
            zip_info.date_time = time.localtime(time.time())[:6]
            zip_info.compress_type = zipfile.ZIP_DEFLATED
            with open(fdir+f, 'rb') as fd:
                zip_file.writestr(zip_info, fd.read())
    fileobj.seek(0)
    return fileobj


def outputs_to_dates(files):
    dates = [datetime.strptime(f, datetime_format) for f in files]
    dates.sort()
    return dates


def date_to_output(d):
    return './out/'+d.strftime(datetime_format)+'.csv'


class ValueRange (tuple):
    def __new__(self, min_v, max_v):
        self.cur_val = None
        self.min_val = min_v
        self.max_val = max_v
        return tuple.__new__(ValueRange, (min_v, max_v))

    def __str__(self):
        return '['+str(self.min_val)+', '+str(self.max_val)+']'

    def init_value(self, base=0):
        self.cur_val = base + round(
            random.uniform(self.min_val, self.max_val), 3)

    def current_value(self):
        return self.cur_val

    def set_value(self, new_v):
        self.cur_val = new_v


def output_path():
    return './out/'+startup_time+'.csv'


def log_path():
    return './log/'+startup_time+'.log'


def parse_value_range(cnfg):

    if not isinstance(cnfg, dict):
        return
    if 'value range' in cnfg:

        rng = cnfg['value range']
        if rng:
            cnfg['value range'] = ValueRange(rng[0], rng[1])

    for cc in cnfg:
        parse_value_range(cnfg[cc])


def load_config(auto_update=False):
    with open('carfi.config', 'r') as config_file:
        config = json.load(config_file)
        for k in config:
            parse_value_range(config[k])

    if auto_update:
        config['scenario name']['value list'] = [name for name, value in getmembers(
            sim.Scenario) if isfunction(value) and not name.endswith('__')]

        config['measures']['choice list'] = [name for name, value in getmembers(
            analysis) if isfunction(value) and not name.endswith('__')]
        config['fault type']['value list'] = [name for name, value in getmembers(
            fa_lib) if isfunction(value) and not name.endswith('__')]

    return config


def get_config_val(val_obj):
    if isinstance(val_obj, dict) and 'value range' in val_obj:
        rng = val_obj['value range']
        if rng:
            return rng.current_value()
        return val_obj['value']
    return val_obj


def init_config_val(val_obj, base=0):
    if isinstance(val_obj, dict) and 'value range' in val_obj:
        rng = val_obj['value range']
        if rng:
            rng.init_value(base)
        else:

            val_obj['value'] += base


def update_config_val(val_obj, new_v):
    if isinstance(val_obj, dict) and 'value range' in val_obj:
        rng = val_obj['value range']
        if rng:
            rng.set_value(new_v)
        else:
            val_obj['value'] = new_v
    else:
        val_obj = new_v


def save_config():
    config = get_config()
    with open('carfi.config', 'w') as config_file:
        json.dump(config, config_file, indent=4)


def extract_fault_params(fault_name):
    src = getsource(getattr(fa_lib, fault_name))
    return set(re.findall(r'fault\[\'([A-Za-z0-9\s_]+)', src))


def extract_scenario_params(scenario_name):
    src = getsource(getattr(sim.Scenario, scenario_name))
    return set(re.findall('config\[\'scenario params\'\]\[\'([A-Za-z0-9\s_]+)', src))


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


def init_logger():
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)
    file_handler = logging.FileHandler(log_path())

    formatter = CustomJsonFormatter('%(DateTime)s %(Level)s')

    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

    return logger
