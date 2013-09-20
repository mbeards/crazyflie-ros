#!/usr/bin/env python

import rospy
from std_msgs.msg import *
from sensor_msgs.msg import *
from crazyflie.msg import *

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.utils import callbacks
from cfclient.utils.logconfigreader import LogConfig
from cfclient.utils.logconfigreader import LogVariable

from threading import Thread

class JoyAdapter:


  def __init__(self):
    rospy.Subscriber("joy", Joy, self.joy_sub_callback)
    self.ori_pub = rospy.Publisher("/crazyflie/OrientationSetPoint", Orientation)

  def joy_sub_callback(self, data):
    pitch = data.axes[1]*90
    yaw = data.axes[2]*90
    roll = data.axes[0]*90
    thrust = 60000*(data.axes[3])
    self.ori_pub.publish(pitch, yaw, roll, thrust)

if __name__ == '__main__':
  rospy.init_node('JoyAdapter')

  joy_adapter = JoyAdapter()

  while not rospy.is_shutdown():
    rospy.sleep(1.0)
