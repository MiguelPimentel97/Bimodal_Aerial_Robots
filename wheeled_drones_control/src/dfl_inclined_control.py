#!/usr/bin/env python

import rospy
import numpy as np
import math
import common

from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, euler_matrix

class DFL_Inclined_Control():

    def __init__(self, position_gains, attitude_gains, inertia, mass, A_inv):

        self.position_gains = position_gains
        self.attitude_gains = attitude_gains
        self.Ix = inertia[0]
        self.Iy = inertia[1]
        self.Iz = inertia[2]
        self.mass = mass
        self.A_inv = A_inv
        self.g = 9.82

        self.clean_errors()

    def clean_errors(self):

        self.e_int = 0
        self.ev_int = 0
        self.ev_last = 0

    def position_stabilization(self, position, velocity, orientation, pos_d, dt):
        # Outer-loop control that regulates position and sends attitude commands to the inner-loop

        # Current State
        z = position[2]
        v = np.sign(velocity[2])*np.sqrt(velocity[0]**2 + velocity[2]**2)

        # Desired State
        z_d = pos_d[2] # Desired altitude
        gamma = pos_d[3] # Inclination
        psi_0 = pos_d[4] # Orientation of slope
        
        # Position Error in Rolling Frame
        e = (z_d - z)/math.sin(abs(gamma))
        self.e_int = common.saturate_value(self.e_int + e*dt, max_value=0.1, min_value=-0.1)
    
        # Desired velocity proportional to error in Rolling Frame
        kp_pos = self.position_gains[0]
        kp_int = self.position_gains[2]
        v_d = kp_pos*e + kp_int*self.e_int

        # Velocity Error in Rolling Frame
        ev = v_d - v
        # Velocity Integral Error
        self.ev_int = self.ev_int + ev*dt

        # Desired acceleration proportional to velocity error
        kp_v = self.position_gains[3]
        kd_v = self.position_gains[4]
        ki_v = self.position_gains[5]
        a_d = kp_v*ev + ki_v*self.ev_int

        # Saturate value and define acceleration valve
        a_max = 0
        a_min = -2*self.g*math.sin(abs(gamma))
        if a_d > a_max:
            delta_a = a_d - a_max
        elif a_d < a_min:
            delta_a = a_d - a_min
        else:
            delta_a = 0

        a_d = common.saturate_value(a_d, max_value=a_max, min_value=a_min)
        self.F0 = self.mass*self.g*math.sin(abs(gamma)) #[N] Equilibrium Thrust
        
        # Desired pitch angle to achieve velocity
        if gamma > 0:
            theta_d = math.asin(common.saturate_value(self.mass/self.F0*(a_d + self.g*math.sin(abs(gamma))), max_value = 1, min_value = -1))
            theta_max = math.pi/3
            theta_min = 0
        else:
            theta_d = -math.asin(common.saturate_value(self.mass/self.F0*(a_d + self.g*math.sin(abs(gamma))), max_value = 1, min_value = -1))
            theta_max = 0
            theta_min = -math.pi/3

        # Saturate value and define pitch valve
        if theta_d > theta_max:
            delta_theta = theta_d - theta_max
        elif theta_d < theta_min:
            delta_theta = theta_d - theta_min
        else:
            delta_theta = 0
        theta_d = common.saturate_value(theta_d, max_value=theta_max, min_value=theta_min)

        # Input Thrust Valve control
        k_a_f = self.position_gains[6]
        k_theta_f = self.position_gains[7]
        self.F = self.F0 + k_a_f*delta_a + k_theta_f*delta_theta
        self.F = common.saturate_value(self.F, min_value=self.mass*self.g/2, max_value=self.mass*self.g)

        attitude = [0, theta_d, psi_0]
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