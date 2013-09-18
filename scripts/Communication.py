#!/usr/bin/env python

import rospy
from crazyfly.msg import *

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.utils import callbacks
from cfclient.utils.logconfigreader import LogConfig
from cfclient.utils.logconfigreader import LogVariable

class Communication:


  def __init__(self):

    self.crazyflie = Crazyflie()
    cflib.crtp.init_drivers()

    available_radios = cflib.crtp.scan_interfaces()

    radio = available_radios[0][0]

    self.crazyflie.open_link(radio)

    self.crazyflie.connectSetupFinished.add_callback(self.connectSetupFinished)
    self.crazyflie.connectionFailed.add_callback(self.connectionLost)
    self.crazyflie.connectionLost.add_callback(self.connectionLost)


  def connectSetupFinished(self, linkURI):
    rospy.loginfo("Connected on %s"%linkURI)

    #Set up pitch/yaw/roll logging (should probably be in a separate function)
    #period_msec = 10

    #self.logconf = LogConfig("Logging", period_msec)

  def connectionLost(self, linkURI):
    rospy.loginfo("Connection lost on %s"%linkURI)
    
  def connectionFailed(self, linkURI):
    rospy.logerr("Connection failed on %s"%linkURI)
