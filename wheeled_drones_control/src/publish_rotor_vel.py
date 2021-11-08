#!/usr/bin/env python

import rospy
import numpy as np
from mav_msgs.msg import Actuators

def talker_humm():
    pub = rospy.Publisher('/hummingbird/command/motor_speed', Actuators, queue_size=1)
    rospy.init_node('publish_motor_speeds')
    rate = rospy.Rate(10)

    pub_object = Actuators()
    pub_object.angular_velocities = [457, 458, 459, 458]

    while not rospy.is_shutdown():
        pub.publish(pub_object)
        rate.sleep()

def talker_crazy():
    pub = rospy.Publisher('/crazywheel/command/motor_speed', Actuators, queue_size=10)
    rospy.init_node('publish_motor_speeds')
    rate = rospy.Rate(10)

    pub_object = Actuators()
    pub_object.angular_velocities = [700, 500, 500, 700]

    while not rospy.is_shutdown():
        pub.publish(pub_object)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker_humm()
    except rospy.ROSInterruptException:
        pass
