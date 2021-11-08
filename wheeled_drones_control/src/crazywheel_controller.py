#!/usr/bin/env python

import rospy
import numpy as np
import math
import yaml
import time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from mav_msgs.msg import RollPitchYawrateThrust, RollPitchYawrateThrustCrazyflie
from tf.transformations import euler_from_quaternion, euler_matrix

class crazywheel_control():

    def __init__(self):

        # Initialize Node and parameters
        rospy.init_node('crazywheel_node', anonymous=True)
        self.init_params()
        self.initialize_variables()
        self.rate = rospy.Rate(100)

        # Initialize pose subscriber from gazebo 
        rospy.Subscriber('/crazywheel/ground_truth/odometry', Odometry, self.PoseCallback)
        # Initialize goal subscriber
        rospy.Subscriber('/desired_position', PoseStamped, self.GoalCallback)

        # Initialize commands publisher 
        self.pub = rospy.Publisher('/crazywheel/command/roll_pitch_yawrate_thrust', RollPitchYawrateThrustCrazyflie, queue_size=1)
        self.pub_object = RollPitchYawrateThrustCrazyflie() # Publisher object

        # Spin Controller
        rospy.spin()

    def init_params(self):

        # Define constants of crazywheel from rosparam
        self.mass = 0.038 
        Ix = 7.9642e-05
        Iy = 5.0608e-05
        Iz = 9.2332e-05
        self.inertia = [Ix, Iy, Iz]

        self.kp_x = 1.5
        self.kp_v = 2.5
        self.kp_psi = 1.2

    def initialize_variables(self):

        self.pos_d = [2, 0]
        self.g = 9.81 
        self.F = self.mass*self.g*0.4

        self.last_time = None
        self.time_offset = rospy.get_time()
        self.x_last = 0
        self.y_last = 0
        self.z_last = 0

    def GoalCallback(self, msg):
        # Function that is called everytime a new waypoint is received

        x_d = msg.pose.position.x
        y_d = msg.pose.position.y
        z_d = msg.pose.position.z

        gamma = msg.pose.orientation.y
        psi_d = msg.pose.orientation.z
 
        self.pos_d = [x_d, y_d, z_d, gamma, psi_d]

    def PoseCallback(self, msg):
        # Function that is called everytime a new odometry message is received

        self.get_time()

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        position = [x, y, z]

        #rospy.loginfo('Current Position: %s', position)

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        q = [qx, qy, qz, qw]
        orientation = euler_from_quaternion(q)
        
        #rospy.loginfo('Current Orientation: %s', orientation)
        
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        linear_velocity = [vx, vy, vz, 1]

        # Update PID controller
        att_cmd = self.pid_controller(position, orientation, linear_velocity)
        self.pub_twist(att_cmd)

    def get_time(self):
        # Time-step determination
    
        cur_time = rospy.get_time() - self.time_offset # Current time in seconds
        if self.last_time is None:
            self.last_time = cur_time # Initialization of variable

        self.dt = cur_time - self.last_time # Time step
        self.last_time = cur_time # Keep in memory the previous time step

        #rospy.loginfo("Current Time: %f", cur_time)

    def pid_controller(self, position, orientation, lin_velocity):

        # State
        x = position[0]
        y = position[1]
        z = position[2]
        
        phi = orientation[0]
        theta = orientation[1]
        psi = orientation[2]

        vx = lin_velocity[0]
        v = vx/math.cos(theta)
        
        # Desired State
        x_d = self.pos_d[0]
        y_d = self.pos_d[1]
        
        # Position Error in World Frame
        ex_i = x_d - x
        ey_i = y_d - y
    
        # Position Error in Rolling Frame
        ex = math.cos(psi)*ex_i + math.sin(psi)*ey_i

        #rospy.loginfo("Current Errors: %s", [ex_i, ey_i, ex])

        # Desired velocity proportional to error along x-axis (Rolling Frame)
        v_d = self.kp_x*ex

        # Velocity Error in Rolling Frame
        ev = v_d - v

        # Desired acceleration proportional to velocity error
        a_d = self.kp_v*ev

        # Desired pitch angle to achieve velocity
        theta_d = math.asin(self.saturate_value(a_d*self.mass/self.F, max_value=1, min_value=-1))
        theta_d = self.saturate_value(theta_d, max_value=math.pi/4, min_value=-math.pi/4)

        # Desired Yaw angle 
        psi_d = math.atan2(ey_i, ex_i)
        if v_d < 0:
            psi_d = psi_d - math.pi*np.sign(psi_d)

        # Desired yaw rate       
        if ex < 0.1:
            yaw_rate = 0
        else:
            yaw_rate = self.kp_psi*self.angular_diff(psi, psi_d)

        return [theta_d, yaw_rate]

    def pub_twist(self, att_cmd):
        # Publish Twist Message in the robot topic

        self.pub_object.roll = 0
        self.pub_object.pitch = att_cmd[0]*180/math.pi
        self.pub_object.yaw_rate = att_cmd[1]*180/math.pi
        self.pub_object.thrust = self.F*10000

        self.pub.publish(self.pub_object)
        self.rate.sleep()

    def saturate_value(self, var, max_value=None, min_value= None):

        if max_value != None and var > max_value:
            var = max_value
        if min_value != None and var < min_value:
            var = min_value

        return var

    def angular_diff(self, ang1, ang2):
        # Returns angular difference wrapped to [-pi, pi]

        error = ang2 - ang1
        sin = math.sin(error)
        cos = math.cos(error)

        ang_diff = math.atan2(sin, cos)
        return ang_diff

if __name__ == '__main__':
    try:
        crazywheel_control()
    except rospy.ROSInterruptException:
        pass