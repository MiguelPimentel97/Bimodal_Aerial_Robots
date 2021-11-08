#!/usr/bin/env python

import rospy
import time
import common
import math

from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu

from tf.transformations import euler_from_quaternion
from pid_ground_control import PID_Ground_Control
from pid_flight_control import PID_Flight_Control
from pid_inclined_control import PID_Inclined_Control

class PID_Node():

    def __init__(self):

        rospy.init_node('PID_Node', anonymous=True) # Initialize Controller Node
        self.initialize_parameters() # Initialize Parameters and Variables initial values

        # Initialize Controller Instances
        self.ground_control = PID_Ground_Control(self.ground_position_gains, self.ground_attitude_gains, self.inertia, self.mass, self.A_inv)
        self.flight_control = PID_Flight_Control(self.flight_position_gains, self.flight_attitude_gains, self.inertia, self.mass, self.A_inv)
        self.inclined_control = PID_Inclined_Control(self.inclined_position_gains, self.ground_attitude_gains, self.inertia, self.mass, self.A_inv)

        # Initialize Publisher of Rotor Velocities
        self.pub_object = Actuators()
        self.pub = rospy.Publisher('/' + self.name + '/command/motor_speed', Actuators, queue_size=1)
        self.rate = rospy.Rate(100)

        # Initialize Publisher of torques
        self.pub_cmd = rospy.Publisher('/Torques', Twist, queue_size=1)
        self.pub_cmd_object = Twist()

        # Initialize Planner Subscriber
        PlanSub = rospy.Subscriber('/desired_position', PoseStamped, self.PlanCallback)

        # Initialize Odometry Subscriber
        OdomSub = rospy.Subscriber('/' + self.name +'/odometry_sensor1/odometry', Odometry, self.OdometryCallback, queue_size=1)

        # Initialize IMU Subscriber
        ImuSub = rospy.Subscriber('/wheelbird/imu', Imu, self.ImuCallback)
        
        rospy.spin()

    def initialize_parameters(self):

        # Initialize Parameters from yaml files
        self.read_yaml_files()
        
        # Calculate Allocation Matrix
        self.A_inv = common.calculate_allocation_matrix(self.d, self.Ct, self.Cm)

        # Determine equilibrium motor velocities
        self.g = 9.81
        self.omega_e = self.mass*self.g/(4*self.Ct)

        self.pos_d = [0, 0, 1, 0, 0] # Keep in the origin until receiving first desired waypoint
        self.mode = 'flight' # Keep in flight mode until planner says otherwise
        self.last_time = None
        self.landed_ground = False
        self.t_landed = 0

    def read_yaml_files(self):

        # Parse yaml file with configuration of robot through rosparam server
        configuration = rospy.get_param(rospy.search_param('rotor_configuration'))
        self.name = rospy.get_param(rospy.search_param('name'))

        # Parse yaml file with controller gains through rosparam server
        parsed_gains_flight = rospy.get_param(rospy.search_param('flight_gains'))
        parsed_gains_ground = rospy.get_param(rospy.search_param('ground_gains'))
        parsed_gains_inclined = rospy.get_param(rospy.search_param('inclined_gains'))

        # Take out relevant variables
        self.Ct = configuration['1']['rotor_force_constant']
        self.Cm = configuration['1']['rotor_moment_constant']
        self.d = configuration['1']['arm_length'] 
        self.mass = rospy.get_param(rospy.search_param('mass'))

        I = rospy.get_param(rospy.search_param('inertia'))
        Ix = I['xx']
        Iy = I['yy']
        Iz = I['zz']
        self.inertia = [Ix, Iy, Iz]

        kp_x = parsed_gains_flight['position_proportional']['x']
        kp_y = parsed_gains_flight['position_proportional']['y']
        kp_z = parsed_gains_flight['position_proportional']['z']
        kd_x = parsed_gains_flight['position_derivative']['x']
        kd_y = parsed_gains_flight['position_derivative']['y']
        kd_z = parsed_gains_flight['position_derivative']['z']
        ki_x = parsed_gains_flight['position_integral']['x']
        ki_y = parsed_gains_flight['position_integral']['y']
        ki_z = parsed_gains_flight['position_integral']['z']

        self.flight_position_gains = [kp_x, kp_y, kp_z,
                                    kd_x, kd_y, kd_z,
                                    ki_x, ki_y, ki_z]

        kp_phi = parsed_gains_flight['attitude_proportional']['x']
        kp_theta = parsed_gains_flight['attitude_proportional']['y']
        kp_psi = parsed_gains_flight['attitude_proportional']['z']
        kd_phi = parsed_gains_flight['attitude_derivative']['x']
        kd_theta = parsed_gains_flight['attitude_derivative']['y']
        kd_psi = parsed_gains_flight['attitude_derivative']['z']
        ki_phi = parsed_gains_flight['attitude_integral']['x']
        ki_theta = parsed_gains_flight['attitude_integral']['y']
        ki_psi = parsed_gains_flight['attitude_integral']['z']

        self.flight_attitude_gains = [kp_phi, kp_theta, kp_psi, 
                                    kd_phi, kd_theta, kd_psi,
                                    ki_phi, ki_theta, ki_psi]

        kp_x_g = parsed_gains_ground['position']['p']
        kp_v_g = parsed_gains_ground['velocity']['p']
        ki_v_g = parsed_gains_ground['velocity']['i']

        self.ground_position_gains = [kp_x_g, kp_v_g, ki_v_g]

        kp_theta_g = parsed_gains_ground['attitude_proportional']['y']
        kp_psi_g = parsed_gains_ground['attitude_proportional']['z']
        kd_theta_g = parsed_gains_ground['attitude_derivative']['y']
        kd_psi_g = parsed_gains_ground['attitude_derivative']['z']
        ki_theta_g = parsed_gains_ground['attitude_integral']['y']

        self.ground_attitude_gains = [kp_theta_g, kp_psi_g,
                                    kd_theta_g, kd_psi_g,
                                    ki_theta_g]

        kp_pos_i = parsed_gains_inclined['position']['p']
        kd_pos_i = parsed_gains_inclined['position']['d']
        ki_pos_i = parsed_gains_inclined['position']['i']
        kp_v_i = parsed_gains_inclined['velocity']['p']
        kd_v_i = parsed_gains_inclined['velocity']['d']
        ki_v_i = parsed_gains_inclined['velocity']['i']

        self.inclined_position_gains = [kp_pos_i, kd_pos_i, ki_pos_i,
                                        kp_v_i, kd_v_i, ki_v_i]

    def PlanCallback(self, msg):
        # Function that is called everytime a new waypoint is received

        x_d = msg.pose.position.x
        y_d = msg.pose.position.y
        z_d = msg.pose.position.z

        gamma = msg.pose.orientation.y
        psi_d = msg.pose.orientation.z
 
        self.pos_d = [x_d, y_d, z_d, gamma, psi_d]
        self.mode = msg.header.frame_id

        # Clean integral and derivative errors
        if self.mode == 'flight':
            self.flight_control.clean_errors()
        elif self.mode == 'ground':
            self.ground_control.clean_errors()
        elif self.mode == 'inclined':
            self.inclined_control.clean_errors()
        
    def OdometryCallback(self, msg):
        # Function that is called everytime a new odometry message is received

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        position = [x, y, z]

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        q = [qx, qy, qz, qw]
        orientation = euler_from_quaternion(q)

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        linear_velocity = [vx, vy, vz, 1]
        
        p = msg.twist.twist.angular.x
        q = msg.twist.twist.angular.y
        r = msg.twist.twist.angular.z
        angular_velocity = [p, q, r]

        #rospy.loginfo('Current Position: %s \n Current Orientation: %s', linear_velocity, orientation)

        if self.mode == 'land':
            if self.landed_ground == True:
                self.t_landed += self.dt
            [rotor_velocities, mode, gamma] = common.land_robot(linear_velocity, self.A_inv, self.landed_ground, self.t_landed, self.mass*self.g)
            self.mode = mode
            self.pos_d = [x, y, z, gamma, orientation[2]]
            self.pub_act(rotor_velocities)

        elif self.mode == 'takeoff':
            self.pos_d = [x, y, z+1, 0, orientation[2]]
            self.mode = 'flight'
            
        elif self.mode == 'emergency':
            self.pub_act([0, 0, 0, 0])            

        else:
            self.update_controller(position, orientation, linear_velocity, angular_velocity)
            
    def update_controller(self, position, orientation, linear_velocity, angular_velocity):
        # Update PID controllers
        self.get_time()

        # Select controller according to pretended mode
        if self.mode == 'flight':
            control = self.flight_control
        elif self.mode == 'ground':
            control = self.ground_control
        elif self.mode == 'inclined':
            control = self.inclined_control

        # Position Controller computes reference Attitude and Thrust Force
        self.att_d = control.position_controller(position, linear_velocity, orientation, self.pos_d, self.dt)
        # Attitude Controller tracks reference Attitude with the Input Torques
        self.cmd_torque = control.attitude_controller(orientation, angular_velocity, self.att_d, self.dt)
        # Control Mixer transforms Input Torques into rotor velocities
        rotor_velocities = common.calculate_rotor_velocities(self.cmd_torque, self.A_inv, self.omega_e, self.mode)

        # Publish rotor velocities
        self.pub_act(rotor_velocities)

    def ImuCallback(self, msg):

        if self.mode == 'land':
            az = msg.linear_acceleration.z - self.g
            if az > 1:
                self.t_landed = 0
                self.landed_ground = True
                rospy.loginfo('Landed on a Surface!')
        else:
            self.landed_ground = False
            
    def get_time(self):
        # Time-step determination
        
        cur_time = rospy.get_rostime().to_sec() # Current time
        if self.last_time is None:
            self.last_time = cur_time # Initialization of variable

        self.dt = cur_time - self.last_time # Time step
        self.last_time = cur_time # Keep in memory the previous time step

        #rospy.loginfo("Current Time: %s", cur_time)

    def pub_act(self, rotor_velocities):
        # Publish Actuator Messages (Motor Angular Speeds) in the robot topic

        self.pub_object.angular_velocities = rotor_velocities
        self.pub.publish(self.pub_object)

        self.pub_cmd_object.linear.x = self.cmd_torque[0]
        self.pub_cmd_object.linear.y = self.att_d[1]
        self.pub_cmd_object.linear.z = self.att_d[2]
        self.pub_cmd_object.angular.x = self.cmd_torque[1]
        self.pub_cmd_object.angular.y = self.cmd_torque[2]
        self.pub_cmd_object.angular.z = self.cmd_torque[3]
        self.pub_cmd.publish(self.pub_cmd_object)

        self.rate.sleep()

if __name__ == '__main__':
    try:
        PID_Node()
    except rospy.ROSInterruptException:
        pass