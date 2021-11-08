#!/usr/bin/env python

import rospy
import numpy as np
import math
import common

class PID_Inclined_Control():

    def __init__(self, position_gains, attitude_gains, inertia, mass, A_inv):

        self.position_gains = position_gains
        self.attitude_gains = attitude_gains
        self.Ix = inertia[0]
        self.Iy = inertia[1]
        self.Iz = inertia[2]
        self.mass = mass
        self.A_inv = A_inv
        self.g = 9.81

        self.clean_errors()

    def clean_errors(self):

        self.psi_error_last = 0
        self.error_theta_int = 0
        self.ev_last = 0
        self.ev_int = 0
        self.e_int = 0
        self.e_last = 0

    def position_controller(self, position, velocity, orientation, pos_d, dt):
        # Outer-loop control that regulates position and sends attitude commands to the inner-loop

        # Current State
        z = position[2]
        v = velocity[2]
        theta = orientation[1]
        psi = orientation[2]

        # Desired State
        z_d = pos_d[2] # Desired altitude
        gamma = pos_d[3] # Inclination
        psi_0 = pos_d[4] # Orientation of slope
        
        # Position Error in World Frame
        e = (z_d - z)/math.sin(abs(gamma))
        # Position Integral Error 
        self.e_int = self.e_int + e*dt
        self.e_int = common.saturate_value(self.e_int, max_value = 0.4, min_value = -0.4)
        # Position Derivative Error
        if dt == 0:
            e_der = 0
        else:
            e_der = (e-self.e_last)/dt
        self.e_last = e

        # Desired velocity with PID error control
        kp_pos = self.position_gains[0]
        kd_pos = self.position_gains[1]
        ki_pos = self.position_gains[2]
        v_d = kp_pos*e + kd_pos*e_der + ki_pos*self.e_int

        # Velocity Error in Rolling Frame
        ev = v_d - v
        # Velocity Integral Error
        self.ev_int = self.ev_int + ev*dt
        self.ev_int = common.saturate_value(self.ev_int, max_value = 0.7, min_value = -0.7)
        # Velocity Derivative Error
        if dt == 0:
            ev_der = 0
        else:
            ev_der = (ev-self.ev_last)/dt # Derivative Error
        self.ev_last = ev

        # Desired acceleration found by PID control law
        kp_v = self.position_gains[3]
        kd_v = self.position_gains[4]
        ki_v = self.position_gains[5]
        a_d = kp_v*ev + ki_v*self.ev_int + kd_v*ev_der
        
        # Desired thrust to achieve velocity
        self.F = self.mass*(a_d+self.g*math.sin(abs(gamma))) 
        self.F = common.saturate_value(self.F, max_value=self.mass*self.g, min_value=self.mass*self.g/2)

        if gamma > 0:
            theta_d = math.pi/2 - gamma
        else:
            theta_d = -math.pi/2 - gamma

        attitude = [0, theta_d, psi_0]
        return attitude

    def attitude_controller(self, orientation, angular_velocity, att_d, dt):
        # PID controller for the attitude

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

        # Gains
        kp_theta = self.attitude_gains[0]
        kp_psi = self.attitude_gains[1]
        kd_theta = self.attitude_gains[2]
        kd_psi = self.attitude_gains[3]
        ki_theta = self.attitude_gains[4]

        # Error Definitions
        error_theta = theta_d - theta
        error_psi = common.angular_diff(psi, psi_d)

        # Calculate Yaw Derivative Error
        if dt == 0:
            error_psi_der = 0
        else:
            error_psi_der = (error_psi-self.psi_error_last)/dt # Derivative Error
        self.psi_error_last = error_psi

        # Desired Angular Accelerations on rolling frame
        acc_theta = kp_theta*error_theta + kd_theta*(-q) + ki_theta*self.error_theta_int # PID Pitch Control
        acc_psi = kp_psi*error_psi + kd_psi*error_psi_der # PD Yaw Control

        # Desired Torques in body frame
        torque_phi = -self.Ix*math.sin(theta)*acc_psi
        torque_theta = self.Iy*acc_theta
        torque_psi = self.Iz*math.cos(theta)*acc_psi

        return [self.F, torque_phi, torque_theta, torque_psi]