#!/usr/bin/env python

import rospy
from std_msgs.msg import *
from crazyflie.msg import *

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.utils import callbacks
from cfclient.utils.logconfigreader import LogConfig
from cfclient.utils.logconfigreader import LogVariable

from threading import Thread

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


    self.acc_pub = rospy.Publisher("/crazyflie/Acceleration", Acceleration)
    self.ori_pub = rospy.Publisher("/crazyflie/Orientation", Orientation)

  def connectSetupFinished(self, linkURI):
    rospy.loginfo("Connected on %s"%linkURI)

    #set up logging
    period_msec = 10
    acc_conf = LogConfig("Accel", period_msec)
    acc_conf.addVariable(LogVariable("acc.x", "float"))
    acc_conf.addVariable(LogVariable("acc.y", "float"))
    acc_conf.addVariable(LogVariable("acc.z", "float"))
  
    ori_conf = LogConfig("Stabilizer", period_msec)
    ori_conf.addVariable(LogVariable("stabilizer.pitch", "float"))
    ori_conf.addVariable(LogVariable("stabilizer.yaw", "float"))
    ori_conf.addVariable(LogVariable("stabilizer.roll", "float"))
    ori_conf.addVariable(LogVariable("stabilizer.thrust", "float"))



    #Set up acc logging
    self.acc_log = self.crazyflie.log.create_log_packet(acc_conf)
    self.orientation_log = self.crazyflie.log.create_log_packet(ori_conf)
  

    if (self.acc_log != None):
      self.acc_log.dataReceived.add_callback(self.acc_data_callback)
      self.acc_log.start()
      rospy.loginfo("Succesfully set up acc logging")
    else:
      rospy.logerr("Unable to set up acc logging")

    if (self.orientation_log != None):
      self.orientation_log.dataReceived.add_callback(self.orientation_data_callback)
      self.orientation_log.start()
      rospy.loginfo("Succesfully set up stabilizer logging")
    else:
      rospy.logerr("Unable to set up stabilizer logging")



  def connectionLost(self, linkURI, errmsg):
    rospy.loginfo("Connection lost on %s: %s"%(linkURI, errmsg))
    exit()
    
  def connectionFailed(self, linkURI):
    rospy.logerr("Connection failed on %s"%linkURI)
    exit()

  def acc_data_callback(self, data):
    #print data
    #rospy.loginfo(str(data))
    self.acc_pub.publish(data["acc.x"], data["acc.y"], data["acc.z"])
  
  def orientation_data_callback(self, data):
    #print data["stabilizer.pitch"], data["stabilizer.yaw"], data["stabilizer.roll"], data["stabilizer.thrust"]
    self.ori_pub.publish(data["stabilizer.pitch"], data["stabilizer.yaw"], data["stabilizer.roll"], data["stabilizer.thrust"])
    #rospy.loginfo(str(data))


if __name__ == '__main__':
  rospy.init_node('communication')

  comms = Communication()

  while not rospy.is_shutdown():
    rospy.sleep(1.0)
