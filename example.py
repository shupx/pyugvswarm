#!/usr/bin/env python
# coding:utf-8
# author: Peixuan SHu
# first created on 2023.1.19
# license: GPLv3
# description: example script of pyugvswarm
# python example.py --sim --vis=null --dt=0.1

import os
from pyugvswarm import *

def main(yaml_path):
    ################### Initialize #####################
    swarm = UGVswarm(yaml_path)
    timeHelper = swarm.timeHelper
    allugvs = swarm.allugvs

    ################# Send cmd_vel #####################
    rate = 20 # Hz
    tf = 5 # (s)
    start_time = timeHelper.time()
    while not timeHelper.isShutdown():
        t = timeHelper.time() - start_time
        if t > tf:
            break
        for i, ugv in enumerate(allugvs.ugvs):
            ugv.cmdVelBody(vx=0.1, vy=0.0, w=1.0)
            pos = ugv.position()
            yaw = ugv.yaw()
        timeHelper.sleepForRate(rate)
    # print("press button to stop...")
    # ugvswarm.input.waitUntilButtonPressed()
    # timeHelper.sleep(5.0)

    ##################### Stop all ugvs #################
    allugvs.stop()
    timeHelper.sleep(0.5)
    
    ######### plot figure ############
    timeHelper.plot_data() # plot x-y, yaw-t (sim only)


if __name__ == "__main__":

    # get the absolute path of the folder of this script
    folder = os.path.dirname(os.path.abspath(__file__)) 
    ugvs_yaml_path = folder + '/config/ugvs.yaml'

    main(ugvs_yaml_path)