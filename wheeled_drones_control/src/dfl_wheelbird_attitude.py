#!/usr/bin/env python

import rospy
import numpy as np
import math
import yaml
import time

from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, euler_matrix

class dfl_control():

    def __init__(self):

        rospy.init_node('subscribe_odometry', anonymous=True)
        self.read_yaml_files('/home/miguel/catkin_ws/src/CrazyS/rotors_gazebo/resource/wheelbird.yaml')
        self.calculate_allocation_matrix(self.thrust_constant, self.moment_constant, self.d)
        self.initialize_variables()

        sub = rospy.Subscriber('/wheelbird/odometry_sensor1/odometry', Odometry, self.OdometryCallback)

        self.pub = rospy.Publisher('/wheelbird/command/motor_speed', Actuators, queue_size=1)
        self.rate = rospy.Rate(10)

        rospy.spin()

    def read_yaml_files(self, conf_file):

        # Parse yaml file with configuration of wheelbird
        configuration = open(conf_file)
        parsed_configuration = yaml.load(configuration)

        # Take out relevant variables
        self.thrust_constant = parsed_configuration['rotor_configuration']['1']['rotor_force_constant']
        self.moment_constant = parsed_configuration['rotor_configuration']['1']['rotor_moment_constant']
        self.d = parsed_configuration['rotor_configuration']['1']['arm_length']  
        self.Ix = parsed_configuration['inertia']['xx']
        self.Iy = parsed_configuration['inertia']['yy']
        self.Iz = parsed_configuration['inertia']['zz']
        self.mass = parsed_configuration['mass']

    def calculate_allocation_matrix(self, Ct, Cm, d):
        # Calculate matrix that maps the rotor veocities with the torques and thrust

        reaction_thrust = d/np.sqrt(2)*Ct
        self.A = np.array([[Ct, Ct, Ct, Ct],
                    [-reaction_thrust, -reaction_thrust, reaction_thrust, reaction_thrust],
                    [-reaction_thrust, reaction_thrust, reaction_thrust, -reaction_thrust],
                    [-Cm*Ct, Cm*Ct, -Cm*Ct, Cm*Ct]]) # Allocation Matrix, last line SPECIFIC to rotor_S implementation

        self.A_inv = np.linalg.inv(self.A) # Inverse of the allocation matrix

    def initialize_variables(self):

        self.pos_d = [0, 0, 0]
        self.T = 6
        self.g = 9.81
        self.last_time = None
        self.e_int = 0

    def OdometryCallback(self, msg):
        # Function that is called everytime a new odometry message is received
        # Note: define a variable with every state later on

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        position = [x, y, z]

        rospy.loginfo('Current Position: %s', position)

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        q = [qx, qy, qz, qw]
        orientation = euler_from_quaternion(q)
        rospy.loginfo('Current Orientation: %s', orientation)
        
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z

        v = vx/math.cos(orientation[1])
        rospy.loginfo('Current velocity: %f', v)

        p = msg.twist.twist.angular.x
        q = msg.twist.twist.angular.y
        r = msg.twist.twist.angular.z
        angular_velocity = [p, q, r]

        # Time-step determination
        
        self.cur_time = time.time() # Current time
        if self.last_time is None:
            self.last_time = self.cur_time # Initialization of variable

        self.dt = self.cur_time - self.last_time # Time step
        self.last_time = self.cur_time # Keep in memory the previous time step

        rospy.loginfo('Current time step: %f', self.dt)

        # Update DFL controller
        cmd_torque = self.dfl_controller(position, orientation, v, angular_velocity, self.pos_d)
        self.pub_act(cmd_torque)

    def dfl_controller(self, position, orientation, v, angular_velocity, pos_d):

        # State
        x = position[0]
        y = position[1]
        z = position[2]
        
        phi = orientation[0]
        theta = orientation[1]
        psi = orientation[2]

        p = angular_velocity[0]
        q = angular_velocity[1]
        r = angular_velocity[2]

        # Desired State
        v_d = 0
        v_d_dot = 0
        v_d_2dot = 0
        v_d_3dot = 0

        phi_d_dot = 0
        psi_d_dot = 0
        psi_d_2dot = 0

        # State Derivatives

        phi_dot = p + r*math.tan(theta)
        theta_dot = q
        psi_dot = r/math.cos(theta)

        v_dot = self.T/self.mass*math.sin(theta)
        v_2dot = self.T/self.mass*q*math.cos(theta)

        # Tracking errors

        e_phi = phi_d - phi
        e_phi_1 = phi_d_dot - phi_dot

        e_v = v_d - v
        e_v_1 = v_d_dot - v_dot
        e_v_2 = v_d_2dot - v_2dot

        e_psi_1 = psi_d_dot - psi_dot

        # Controller Gains
        cx0=19
        cx1=19
        cv0=40
        cv1=10
        cv2=10
        cp1=10

        # Tracking Controller
        v1 = cx1*e_phi_1 + cx0*e_phi
        v2 = v_d_3dot + cv2*e_v_2 + cv1*e_v_1 + cv0*e_v
        v3 = psi_d_2dot + cp1*e_psi_1
        u_tilde = np.array([v1, v2, v3])

        # Dynamic Compensator
        l1 = q*r/(math.cos(theta)**2)
        l2 = -self.T/self.mass*q**2*math.sin(theta)            
        l3 = r*math.tan(theta)/math.cos(theta)
        l = np.array([l1, l2, l3]) 

        # Decoupling Matrix
        Jinv = np.array([[self.Ix, 0, -self.Ix*math.sin(theta)],
                        [0, self.Iy/math.cos(theta)*self.mass/self.T , 0],
                        [0, 0, self.Iz*math.cos(theta)]])

        # Control Law
        u = np.dot(Jinv, np.add(u_tilde, -l))

        rospy.loginfo('Commanded Torques: %s', u)

        torque_phi = u[0]
        torque_theta = u[1]
        torque_psi = u[2]

        return [self.T, torque_phi, torque_theta, torque_psi]

    def pub_act(self, cmd_torques):
        # Publish Actuator Messages (Motor Angular Speeds) in the robot topic

        pub_object = Actuators()
        pub_object.angular_velocities = self.calculate_rotor_velocities(cmd_torques)
        self.pub.publish(pub_object)

    def calculate_rotor_velocities(self, cmd_torques):
        # Calculate the required rotor velocities to achieve desired torques and thrust 

        rotor_velocities_squared = np.dot(self.A_inv, cmd_torques) # Get the squared angular velocites (A.wi^2 = Tau)

        # Making sure the rotor velocities are not negative and do not cause the robot to take-off
        for i, vel in enumerate(rotor_velocities_squared):
            if vel > 324900:
                rotor_velocities_squared[i] = 324900
            if vel < 0:
                rotor_velocities_squared[i] = 0

        rotor_velocities = np.sqrt(rotor_velocities_squared)

        rospy.loginfo('Commanded Rotor Velocities: %s', rotor_velocities)
        return rotor_velocities

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
        dfl_control()
    except rospy.ROSInterruptException:
        pass