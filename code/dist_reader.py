#!/usr/bin/env python
#
#
# Copyright
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rospy
from snowmower_msgs.msg import DecaWaveMsg
import message_filters
from message_filters import TimeSynchronizer, Subscriber
from datetime import datetime
import string
import serial, time, os
from serial import SerialException
from decimal import Decimal
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os


bridge = CvBridge()
index = 0
initial_time = 0
range_sec = 0
range_nsec = 0
initial_time = 0.0

last_range0 = 0.0
last_range = 0.0
range0 = 0.0
last_time = 0.0


class DecaWaveDriver:
  def __init__(self):
    rospy.init_node('decawave_driver')
    #Init DecaWave port
    self.dwPort = rospy.get_param('~port','dev/ttyACM0')
    self.dwRate = rospy.get_param('~baud', 115200)
    self.dwID   = rospy.get_param('~id','a0')
    self.anchor_angle = rospy.get_param('~anchor_angle',0)
    self.tag_angle = rospy.get_param('~tag_angle',0)
    self.beacon_distance = rospy.get_param('~beacon_distance',0)
    self.dwPub = rospy.Publisher('/ranger_finder/data', DecaWaveMsg, queue_size = 5)

    try:
      ser = serial.Serial(
      port = self.dwPort,
      timeout = 10,
      baudrate = self.dwRate
      )

      dwMsg = DecaWaveMsg()
      dwMsg.header.frame_id = "base_decawave_" + self.dwID

      ser.close()
      ser.open()  
      range0 = 0

      loop_rate = rospy.Rate(3.33)
      time_array = np.array([])
     
      while (not rospy.is_shutdown()):
        raw_data = ser.readline()
        if raw_data == serial.to_bytes([]):
          print "serial timeout"
        else:
          data = raw_data.split(' ')
          if data[0] == 'mc':
            dwMsg.header.stamp = rospy.Time.now()
  	    now = dwMsg.header.stamp
            rospy.loginfo("UW time %f", rospy.get_time())
            mask = int(data[1],16)
            if (mask & 0x01):
              global range0
              range0 = int(data[2],16)/1000.0
              if range0!=0:
                  print range0
                  dwMsg.dist = range0                 
                  self.dwPub.publish(dwMsg)   #publish the topic
            else:
              print "range0 bad"

      ser.close()
      file.flush()
      file.close()
      
      
     
    except SerialException:
      print("Could not connect to the serial port")
    
if __name__ == "__main__":
  dw = DecaWaveDriver()
  
