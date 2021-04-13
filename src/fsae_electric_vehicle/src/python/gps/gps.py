#!/usr/bin/env python
# CSUN FSAE EV 2018
# ROS NODE: GPS
# Topics Subcribed: None
# Topics Publishing: gps_data
# Summary:
#     The purpose of this node is to read latitude and longitude data
#     from the Sierra Wireless MC7455 Modem
# Related:
#     MC7455 Compatible Drivers: (https://source.sierrawireless.com/resources/airprime/software/usb-drivers-linux-qmi-software-latest/)
#     Configurations were set up using Minicom using AT Commands
#     (https://source.sierrawireless.com/resources/airprime/minicard/74xx/4117727-airprime-em74xx-mc74xx-at-command-reference/)
#   Documents:
#      Drivers configuration guide: (https://source.sierrawireless.com/resources/airprime/software/linux-qmi-sdk-application-developer-guide-1,-d-,26/)
#

import serial
import re
#import pynmea2
import rospy
#from datetime import datetime, timezone
from fsae_electric_vehicle.msg import gps

msg = gps()
pub = rospy.Publisher('GPS', gps, queue_size=10)
rospy.init_node('GPS', anonymous=True)
rate = rospy.Rate(1)
def GPS():
    while not rospy.is_shutdown():
        msg.latitude = 3
        msg.longitude = 3
        pub.publish(msg)
        #rospy.spin()
             

if __name__ == '__main__':
    try:
        GPS()
    except rospy.ROSInterruptException:
        pass
