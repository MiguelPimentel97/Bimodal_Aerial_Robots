#!/usr/bin/env python

import rospy
import numpy as np
import math
import common

from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, euler_matrix

class DFL_Ground_Control():

    def __init__(self, position_gains, attitude_gains, inertia, mass, A_inv):

        self.position_gains = position_gains
        self.attitude_gains = attitude_gains
        self.Ix = inertia[0]
        self.Iy = inertia[1]
        self.Iz = inertia[2]
        self.mass = mass
        self.A_inv = A_inv
        self.g = 9.81

        self.F0 = self.mass*self.g*0.6 #[N] 50% of Lift-Off Thrust

    def position_stabilization(self, position, velocity, orientation, pos_d, dt=0):
        # Outer-loop control that regulates position and sends attitude commands to the inner-loop

        # Current State
        x = position[0]
        y = position[1]
        vx = velocity[0]
        theta = orientation[1]
        psi = orientation[2]

        # Forward/Bacwards velocity
        v = vx/math.cos(theta)

        # Desired State
        x_d = pos_d[0]
        y_d = pos_d[1]
        
        # Position Error in World Frame
        ex_i = x_d - x
        ey_i = y_d - y

        # Position Error in Rolling Frame
        ex = math.cos(psi)*ex_i + math.sin(psi)*ey_i
        ey = -math.sin(psi)*ex_i + math.cos(psi)*ey_i
        
        # Desired velocity proportional to error along x-axis (Rolling Frame)
        kp_x = self.position_gains[0]
        v_d = kp_x*ex

        # Velocity Error in Rolling Frame
        ev = v_d - v

        # Desired acceleration proportional to velocity error
        kp_v = self.position_gains[1]
        a_d = kp_v*ev

        # Saturate value and define acceleration valve
        a_max = self.F0/self.mass
        a_min = -a_max
        if a_d > a_max:
            delta_a = a_d - a_max
        elif a_d < a_min:
            delta_a = a_min - a_d
        else:
            delta_a = 0

        a_d = common.saturate_value(a_d, max_value=a_max, min_value=a_min)
        
        # Desired pitch angle to achieve velocity
        theta_d = math.asin(a_d*self.mass/self.F0)

        # Saturate value and define pitch valve
        theta_max = math.pi/3
        theta_min = -theta_max
        if theta_d > theta_max:
            delta_theta = theta_d - theta_max
        elif theta_d < theta_min:
            delta_theta = theta_min - theta_d
        else:
            delta_theta = 0
        theta_d = common.saturate_value(theta_d, max_value=theta_max, min_value=theta_min)

        # Input Thrust Valve control
        k_a_f = self.position_gains[2]
        k_theta_f = self.position_gains[3]
        self.F = self.F0 + k_a_f*delta_a + k_theta_f*delta_theta
        self.F = common.saturate_value(self.F, max_value=self.mass*self.g)

        #rospy.loginfo('Delta: %f, %f', delta_a, delta_theta)

        # Desired Yaw angle 
        psi_d = math.atan2(ey_i, ex_i)
        if v_d < 0:
            psi_d = psi_d - math.pi*np.sign(psi_d)

        # Stop INFINTE ROTATION
        if abs(ex) < 0.01 and abs(ey)<0.01:
            psi_d = psi

        #rospy.loginfo('Desired Yaw Angle: %f \n Desired Pitch Angle: %f \n Input Thrust: %f', psi_d, theta_d, self.F)

        attitude = [0, theta_d, psi_d]
        return attitude

    def dfl_controller(self, orientation, angular_velocity, att_d):

        # State
        phi = orientation[0]
        theta = orientation[1]
        psi = orientation[2]

        p = angular_velocity[0]
        q = angular_velocity[1]
        r = angular_velocity[2]

        # Desired State
        phi_d = att_d[0]
        theta_d = att_d[1]
        psi_d = att_d[2]

        phi_d_dot = 0
        theta_d_dot = 0
        psi_d_dot = 0
        
        phi_d_2dot = 0
        theta_d_2dot = 0
        psi_d_2dot = 0

        # State Derivatives
        phi_dot = p + r*math.tan(theta)
        theta_dot = q
        psi_dot = r/math.cos(theta)

        # Tracking errors
        e_phi = phi_d - phi
        e_phi_1 = phi_d_dot - phi_dot

        e_theta = theta_d - theta
        e_theta_1 = theta_d_dot - theta_dot

        e_psi = common.angular_diff(psi, psi_d)
        e_psi_1 = psi_d_dot - psi_dot

        # Controller Gains
        cx0=self.attitude_gains[0]
        cx1=self.attitude_gains[3]
        cy0=self.attitude_gains[1]
        cy1=self.attitude_gains[4]
        cp0=self.attitude_gains[2]
        cp1=self.attitude_gains[5]

        # Tracking Controller
        v1 = phi_d_2dot + cx1*e_phi_1 + cx0*e_phi
        v2 = theta_d_2dot + cy1*e_theta_1 + cy0*e_theta
        v3 = psi_d_2dot + cp1*e_psi_1  + cp0*e_psi
        v = np.array([v1, v2, v3])

        # Dynamic Compensator
        l1 = (self.Iy-self.Iz)/self.Ix*q*r + (self.Ix-self.Iy)/self.Iz*p*q*math.tan(theta) + q*r/(math.cos(theta)**2)
        l2 = (self.Iz-self.Ix)/self.Iy*p*r            
        l3 = (self.Ix-self.Iy)/self.Iz*p*q/math.cos(theta) + r*math.tan(theta)/math.cos(theta)
        l = np.array([l1, l2, l3]) 

        # Decoupling Matrix
        Jinv = np.array([[self.Ix, 0, -self.Ix*math.sin(theta)],
                        [0, self.Iy, 0],
                        [0, 0, self.Iz*math.cos(theta)]])

        # Control Law
        u = np.dot(Jinv, np.add(v, -l))

        #rospy.loginfo('Commanded Torques: %s', u)

        torque_phi = u[0]
        torque_theta = u[1]
        torque_psi = u[2]

        return [self.F, torque_phi, torque_theta, torque_psi]