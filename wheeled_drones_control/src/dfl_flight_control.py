#!/usr/bin/env python

import rospy
import numpy as np
import math
import common

from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, euler_matrix

class DFL_Flight_Control():

    def __init__(self, position_gains, attitude_gains, inertia, mass, A_inv):

        self.position_gains = position_gains
        self.attitude_gains = attitude_gains
        self.Ix = inertia[0]
        self.Iy = inertia[1]
        self.Iz = inertia[2]
        self.mass = mass
        self.A_inv = A_inv
        self.g = 9.81 # Gravity acceleration
        self.F = mass*9.81

    def position_stabilization(self, position, velocity, orientation, pos_d, dt=0):

        # State

        x = position[0]
        y = position[1]
        z = position[2]

        phi = orientation[0]
        theta = orientation[1]
        psi = orientation[2]
        R = euler_matrix(phi, theta, psi, axes='sxyz')

        # Since linear velocity is in body frame, we need to rotate it to inertial frame
        linear_velocity = np.dot(R, np.transpose(velocity))
        vx = linear_velocity[0]
        vy = linear_velocity[1]
        vz = linear_velocity[2]

        # Desired State

        x_d = pos_d[0]
        y_d = pos_d[1]
        z_d = pos_d[2]
        psi_d = pos_d[4]

        # Gains
        kp_x = self.position_gains[0]
        kp_y = self.position_gains[1]
        kd_x = self.position_gains[2]
        kd_y = self.position_gains[3]
        kp_z = self.attitude_gains[0]
        kd_z = self.attitude_gains[4]

        # Position Error in World Frame
        e_x = x_d - x
        e_y = y_d - y
        e_z = z_d - z

        # Altitude Stabilization: PID controller
        Fz = kp_z*e_z + kd_z*(-vz)
        self.F = self.mass/(math.cos(phi)*math.cos(theta))*(self.g + Fz) # Cancel Non-linearities

        ## Position Stabilization: PD control laws 

        # Desired thrust direction
        Fx = (self.mass/self.F)*(kp_x*e_x + kd_x*(-vx))
        Fy = (self.mass/self.F)*(kp_y*e_y + kd_y*(-vy))

        # Desired Attitude to achieve thrust direction
        phi_d = math.asin(common.saturate_value(Fx*math.sin(psi)-Fy*math.cos(psi), max_value=1, min_value=-1))
        phi_d = common.saturate_value(phi_d, max_value=math.pi/6, min_value=-math.pi/6)

        theta_d = math.asin(common.saturate_value((Fx*math.cos(psi)+Fy*math.sin(psi))/math.cos(phi_d), max_value=1, min_value=-1))
        theta_d = common.saturate_value(theta_d, max_value=math.pi/6, min_value=-math.pi/6)

        attitude = [phi_d, theta_d, psi_d]
        #rospy.loginfo('Desired Attitude: %s', attitude)
        return attitude

    def dfl_controller(self, orientation, angular_velocity, att_d):

        # Reference commands

        phi_d = att_d[0]
        theta_d = att_d[1]
        psi_d = att_d[2]

        phi_dot_d = 0
        theta_dot_d = 0
        psi_dot_d = 0

        # State

        phi = orientation[0]
        theta = orientation[1]
        psi = orientation[2]

        p = angular_velocity[0]
        q = angular_velocity[1]
        r = angular_velocity[2]

        # State Derivatives

        sph = math.sin(phi)
        cph = math.cos(phi)
        sth = math.sin(theta)
        cth = math.cos(theta)
        tth = math.tan(theta)

        phi_dot = p + sph*tth*q + cph*tth*r
        theta_dot = cph*q - sph*r
        psi_dot = (sph*q + cph*r)/cth

        # Tracking errors

        e_phi = phi_d - phi
        e_phi_1 = phi_dot_d - phi_dot

        e_theta = theta_d - theta
        e_theta_1 = theta_dot_d - theta_dot

        e_psi = common.angular_diff(psi, psi_d)
        e_psi_1 = psi_dot_d - psi_dot

        # Controller Gains

        cx0=self.attitude_gains[1]
        cx1=self.attitude_gains[5]
        cy0=self.attitude_gains[2] 
        cy1=self.attitude_gains[6] 
        cp0=self.attitude_gains[3]
        cp1=self.attitude_gains[7]

        # Tracking Controller

        v2 = cx1*e_phi_1 + cx0*e_phi
        v3 = cy1*e_theta_1 + cy0*e_theta
        v4 = cp1*e_psi_1 + cp0*e_psi

        v = np.array([v2, v3, v4])

        # Dynamic Compensator

        p_dot = (self.Iy-self.Iz)/self.Ix*q*r
        q_dot = (self.Iz-self.Ix)/self.Iy*p*r
        r_dot = (self.Ix-self.Iy)/self.Iz*p*q

        l2 = p_dot + q_dot*sph*tth + r_dot*cph*tth + phi_dot*tth*(cph*q-sph*r) + theta_dot*(sph*q+cph*r)/(cth*cth)
                
        l3 = q_dot*cph - r_dot*sph - phi_dot*(sph*q+cph*r)
            
        l4 = q_dot*sph/cth + r_dot*cph/cth + phi_dot*(cph*q-sph*r)/cth + theta_dot*tth*(sph*q+cph*r)/cth

        l = np.array([-l2, -l3, -l4]) 

        # Decoupling Matrix

        Jinv = np.array([[self.Ix, 0, -self.Ix*sth],
                        [0, self.Iy*cph, self.Iy*cth*sph],
                        [0, -self.Iz*sph, self.Iz*cph*cth]])

        # Control Law

        u = np.dot(Jinv, np.add(v, l))

        #rospy.loginfo('Commanded Torques: %s', u)

        torque_phi = u[0]
        torque_theta = u[1]
        torque_psi = u[2]

        return [self.F, torque_phi, torque_theta, torque_psi]