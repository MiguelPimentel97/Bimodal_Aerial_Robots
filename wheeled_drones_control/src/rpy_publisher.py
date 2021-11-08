#!/usr/bin/env python

import rospy
from mav_msgs.msg import RollPitchYawrateThrust, RollPitchYawrateThrustCrazyflie


def talker():
    pub = rospy.Publisher('/wheelbird/command/roll_pitch_yawrate_thrust', RollPitchYawrateThrust, queue_size=1)
    rospy.init_node('publish_command')
    rate = rospy.Rate(10)

    pub_object = RollPitchYawrateThrust()
    pub_object.roll = 0
    pub_object.pitch = 0.2
    pub_object.yaw_rate = 0
    pub_object.thrust.x = 0
    pub_object.thrust.y = 0
    pub_object.thrust.z = 4

    while not rospy.is_shutdown():
        pub.publish(pub_object)
        rate.sleep()

def talker_crazy():
    pub = rospy.Publisher('/crazywheel/command/roll_pitch_yawrate_thrust', RollPitchYawrateThrustCrazyflie, queue_size=1)
    rospy.init_node('publish_command')
    rate = rospy.Rate(10)

    pub_object = RollPitchYawrateThrustCrazyflie()
    pub_object.roll = 0
    pub_object.pitch = 30
    pub_object.yaw_rate = -0.8
    pub_object.thrust = 2000

    while not rospy.is_shutdown():
        pub.publish(pub_object)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker_crazy()
    except rospy.ROSInterruptException:
        pass






