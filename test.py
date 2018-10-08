#!/usr/bin/env python

try:
    import numpy as np
except:
    raise ImportError("Numpy module not installed.")
try:
    import yaml
except:
    raise ImportError("Yaml module not installed.")
import time
import sys
sys.path.append("./Hive")
from Hive import Hive
from Hive import Utilities
import Evaluator as eval

def run():
    f=open('./config.yml')
    config=yaml.load(f)
    roi_shape = config['roi_shape']
    dead_zone = config['dead_zone']
    cube_resolution = config['cube_resolution']
    laser_num = config['laser_num']
    lidar_num = config['lidar_num']
    beam_angle = config['beam_angle']
    max_itrs = config['max_itrs']
    numb_bees = config['numb_bees']
    lower_bound = config['lower_bound'] * lidar_num
    upper_bound = config['upper_bound'] * lidar_num
    estimate_solve_time = config['estimate_solve_time']
    save_result_to_json = config['save_result_to_json']

    assert (len(beam_angle)==laser_num), "Please check the number of beams' angles of pitch"
    eval.logConfig(config=config)

    VSR_solver = eval.Evaluator(roi_shape=roi_shape, dead_zone=dead_zone,cube_resolution=cube_resolution,
                                             laser_num=laser_num, lidar_num=lidar_num,pitch_angle=beam_angle)
    now = time.time()
    for i in range(max_itrs):

        VSR_solver.solve(lower_bound)

    total_time=time.time() - now
    print("Average time consume: ",total_time/max_itrs)


if __name__ == "__main__":
    run()
