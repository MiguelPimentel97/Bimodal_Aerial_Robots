#!/usr/bin/env python

import rospy
import numpy as np
import sys
from geometry_msgs.msg import PoseStamped

def path_publish():

    pub = rospy.Publisher('/desired_position', PoseStamped, queue_size=1)
    rospy.init_node('publish_point') # Initialize Node
    rate = rospy.Rate(10)

    pub_object = PoseStamped()
    mode = sys.argv[1]
    pub_object.header.frame_id = mode

    if mode == 'land':
        print('Landing Requested!')

    elif mode == 'emergency':
        print('Emergency Requested!')

    elif mode == 'takeoff':
        print('Taking Off!')

    elif mode == 'flight':

        if len(sys.argv) == 6:
            pub_object.pose.position.x = float(sys.argv[2])
            pub_object.pose.position.y = float(sys.argv[3])
            pub_object.pose.position.z = float(sys.argv[4])
            pub_object.pose.orientation.y = 0
            pub_object.pose.orientation.z = float(sys.argv[5])
        else:
            print('Wrong number of arguments \n (...).py flight x y z psi')
    
    elif mode == 'ground':

        if len(sys.argv) == 4:
            pub_object.pose.position.x = float(sys.argv[2])
            pub_object.pose.position.y = float(sys.argv[3])
            pub_object.pose.position.z = 0
            pub_object.pose.orientation.y = 0
            pub_object.pose.orientation.z = 0
        else:
            print('Wrong number of arguments \n (...).py ground x y')

    elif mode == 'inclined':

        if len(sys.argv) == 5:
            pub_object.pose.position.x = 0
            pub_object.pose.position.y = 0
            pub_object.pose.position.z = float(sys.argv[2])
            pub_object.pose.orientation.y = float(sys.argv[3])
            pub_object.pose.orientation.z = float(sys.argv[4])
        else:
            print('Wrong number of arguments \n (...).py inclined z gamma psi0')

    else:
        print('Wrong number of arguments or incorrect use')
        print('Correct use: \n (...).py flight x y z psi \n (...).py ground x y \n (...).py inclined z gamma psi0')
        
    pub.publish(pub_object)
    rate.sleep()

if __name__ == '__main__':
    try:
        path_publish()
    except rospy.ROSInterruptException:
        pass
