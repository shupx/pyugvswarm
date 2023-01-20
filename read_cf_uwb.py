#!/usr/bin/env python3
# coding:utf-8

# Peixuan Shu
# First created on 5 Jan. 2023
# Read USB connected crazyflie UWB pose by cflib and publish it in a ROS topic
# Note: cflib is in python3:
#   pip3 install cflib  # should > 0.1.20
# Get UGV pos from the attached crazyflie uwb:

import logging

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

import rospy
import geometry_msgs.msg


# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='usb://0')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def log_stab_callback(timestamp, data, logconf):
    # print('[%d][%s]: %s' % (timestamp, logconf.name, data))
    msg = geometry_msgs.msg.PoseStamped()
    msg.pose.position.x = data['stateEstimateZ.x'] # mm
    msg.pose.position.y = data['stateEstimateZ.y'] # mm
    msg.pose.position.z = data['stateEstimateZ.z'] # mm
    PosePublisher.publish(msg)


def simple_log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_stab_callback)
    logconf.start()
    # time.sleep(5)
    # logconf.stop()
    rospy.spin() # block for other threads active

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    lg_stab = LogConfig(name='stateEstimateZ', period_in_ms=20)
    lg_stab.add_variable('stateEstimateZ.x', 'int16_t') # mm
    lg_stab.add_variable('stateEstimateZ.y', 'int16_t') # mm
    lg_stab.add_variable('stateEstimateZ.z', 'int16_t') # mm

    # group = 'stateEstimateZ'
    # name = 'estimator'

    rospy.init_node("read_cf_uwb", anonymous=False)
    topic_name = "/car20/pos"
    PosePublisher = rospy.Publisher(topic_name, geometry_msgs.msg.PoseStamped, queue_size=1)

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        print("Yeah, I'm connected!")
        print("check {0} streaming!".format(topic_name))
        simple_log_async(scf, lg_stab)
