#!/usr/bin/python2

import rospy
from fsae_electric_vehicle.msg import gps
from fsae_electric_vehicle.msg import serial
from fsae_electric_vehicle.msg import speedometer
from fsae_electric_vehicle.msg import brake_pressure
from fsae_electric_vehicle.msg import coolant
from fsae_electric_vehicle.msg import drive_train

def callback(data):
    rospy.loginfo("%d the latitude value: %d the longitude value" % (data.latitude, data.longitude))
    print(data.latitude)

def callbackspeed(data):
    rospy.loginfo("%f the speed value %d" % (data.speed, 42))
    
def callbackbrake(data):
    rospy.loginfo("%f brake pressure value %d" % (data.pressure, 33))
    
def callbackcoolant(data):
    rospy.loginfo("%f coolant temp value %d" % (data.temperature, 12))
    
def callbackdrivetrain(data):
    rospy.loginfo("%f voltage value %d" % (data.voltage, 99))

def listener():
    rospy.init_node('custom_listener', anonymous=True)
    rospy.Subscriber("GPS", gps, callback)
    print('hello!')

    rospy.Subscriber("speedometer", speedometer, callbackspeed)
    rospy.Subscriber("brake_pressure", brake_pressure, callbackbrake)
    rospy.Subscriber("coolant_temperature", coolant, callbackcoolant)
    rospy.Subscriber("drivetrain_voltage", drive_train, callbackdrivetrain)

    #pub = rospy.Publisher('serial', serial)

     #msg = serial()
     #msg.latitude = 0
     #rate = rospy.Rate(5)

     #while not rospy.is_shutdown:
     #    msg.latitude = msg.latitude + 1
     #    pub.publish(msg)
     #    rate.sleep()


     #pub.publish()

   # spin() simply keeps python from exiting until this node is stopped

    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
