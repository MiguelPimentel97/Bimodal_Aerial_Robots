#!/usr/bin/env python

import rospy
import numpy as np
import math
import common

from tf.transformations import euler_matrix

class PID_Flight_Control():

    def __init__(self, position_gains, attitude_gains, inertia, mass, A_inv):

        self.position_gains = position_gains
        self.attitude_gains = attitude_gains
        self.Ix = inertia[0]
        self.Iy = inertia[1]
        self.Iz = inertia[2]
        self.mass = mass
        self.A_inv = A_inv
        self.g = 9.81 # Gravity acceleration

        self.clean_errors()

    def clean_errors(self):

        self.ex_int = 0
        self.ey_int = 0
        self.ez_int = 0
        self.e_phi_int = 0
        self.e_theta_int = 0
        self.e_psi_int = 0

    def position_controller(self, position, velocity, orientation, pos_d, dt):
        # Outer-loop control that regulates position and sends attitude commands to the inner-loop

        # Current State
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
        kp_z = self.position_gains[2]
        kd_x = self.position_gains[3]
        kd_y = self.position_gains[4]
        kd_z = self.position_gains[5]
        ki_x = self.position_gains[6]
        ki_y = self.position_gains[7]
        ki_z = self.position_gains[8]

        # Position Error in World Frame
        e_x = x_d - x
        e_y = y_d - y
        e_z = z_d - z

        # Integral Error in Z-direction
        self.ez_int = self.ez_int + e_z*dt 

        # Altitude Stabilization: PID controller
        Uz = kp_z*e_z + kd_z*(-vz) + ki_z*self.ez_int
        F = self.mass*(self.g + Uz) # Cancel Non-linearities
        
        # Desired accelerations: PD control laws
        Ux = (kp_x*e_x + kd_x*(-vx))
        Uy = (kp_y*e_y + kd_y*(-vy))

        # Desired Attitude to achieve thrust direction

        phi_d = (Ux*math.sin(psi_d)-Uy*math.cos(psi_d))/self.g
        phi_d = common.saturate_value(phi_d, max_value=math.pi/12, min_value=-math.pi/12)

        theta_d = (Ux*math.cos(psi_d)+Uy*math.sin(psi_d))/self.g
        theta_d = common.saturate_value(theta_d, max_value=math.pi/12, min_value=-math.pi/12)

        attitude = [phi_d, theta_d, psi_d, F]
        return attitude

    def attitude_controller(self, orientation, angular_velocity, att_d, dt):
        # Inner-Loop that tracks reference attitude commands

        # Current attitude
        phi = orientation[0]
        theta = orientation[1]
        psi = orientation[2]

        p = angular_velocity[0]
        q = angular_velocity[1]
        r = angular_velocity[2]

        # Desired attitude
        phi_d = att_d[0]
        theta_d = att_d[1]
        psi_d = att_d[2]

        # Thrust Force
        F = att_d[3]

        # Gains
        kp_phi = self.attitude_gains[0]
        kp_theta = self.attitude_gains[1]
        kp_psi = self.attitude_gains[2]
        kd_phi = self.attitude_gains[3]
        kd_theta = self.attitude_gains[4]
        kd_psi = self.attitude_gains[5]
        ki_phi = self.attitude_gains[6]
        ki_theta = self.attitude_gains[7]
        ki_psi = self.attitude_gains[8]        

        # Error Definitions
        e_phi = phi_d - phi
        e_theta = theta_d - theta
        e_psi = common.angular_diff(psi, psi_d) 

        # Desired Angular Accelerations on rolling frame: PD control laws
        acc_phi = kp_phi*e_phi + kd_phi*(-p) 
        acc_theta = kp_theta*e_theta + kd_theta*(-q)
        acc_psi = kp_psi*e_psi + kd_psi*(-r) 

        # Desired Torques in body frame
        torque_phi = self.Ix*acc_phi
        torque_theta = self.Iy*acc_theta
        torque_psi = self.Iz*acc_psi

        return [F, torque_phi, torque_theta, torque_psi]