#!/usr/bin/env python

import rospy
import numpy as np
import math
import common

class PID_Ground_Control():

    def __init__(self, position_gains, attitude_gains, inertia, mass, A_inv):

        self.position_gains = position_gains
        self.attitude_gains = attitude_gains
        self.Ix = inertia[0]
        self.Iy = inertia[1]
        self.Iz = inertia[2]
        self.mass = mass
        self.A_inv = A_inv

        self.F = mass*9.81*0.6 #[N] # 60% of lift-off thrust force
        self.clean_errors()

    def clean_errors(self):

        self.error_theta_int = 0
        self.ev_int = 0
        self.psi_error_last = 0

    def position_controller(self, position, velocity, orientation, pos_d, dt):
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
        self.ev_int = self.ev_int + ev*dt # Integral Error

        # Desired acceleration proportional to velocity error
        kp_v = self.position_gains[1]
        ki_v = self.position_gains[2]
        a_d = kp_v*ev + ki_v*self.ev_int
        a_d = common.saturate_value(a_d, max_value=self.F/self.mass, min_value=-self.F/self.mass)

        # Desired pitch angle to achieve velocity
        theta_d = math.asin(a_d*self.mass/self.F)
        theta_d = common.saturate_value(theta_d, max_value=math.pi/6, min_value=-math.pi/6)

        # Desired Yaw angle 
        psi_d = math.atan2(ey_i, ex_i)
        if v_d < 0:
            psi_d = psi_d - math.pi*np.sign(psi_d)

        # Stop INFINTE ROTATION
        if abs(ex) < 0.01 and abs(ey)<0.01:
            psi_d = psi

        #rospy.loginfo('Desired Yaw Angle: %f \n Desired Pitch Angle: %f', psi_d, theta_d)

        attitude = [0, theta_d, psi_d]
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

        # Error Definitions
        error_theta = theta_d - theta
        error_psi = common.angular_diff(psi, psi_d)

        # Calculate Yaw Derivative Error
        if dt == 0:
            error_psi_der = 0
        else:
            error_psi_der = (error_psi-self.psi_error_last)/dt # Derivative Error
        self.psi_error_last = error_psi

        # Calculate Pitch Integral Error
        error_theta_int = self.error_theta_int + error_theta*dt
        self.error_theta_int = common.saturate_value(error_theta_int, max_value=math.pi/4, min_value=-math.pi/4)

        # Gains
        kp_theta = self.attitude_gains[0]
        kp_psi = self.attitude_gains[1]
        kd_theta = self.attitude_gains[2]
        kd_psi = self.attitude_gains[3]
        ki_theta = self.attitude_gains[4]

        # Desired Angular Accelerations on rolling frame
        acc_theta = kp_theta*error_theta + kd_theta*(-q) + ki_theta*self.error_theta_int # PID Pitch Control
        acc_psi = kp_psi*error_psi + kd_psi*error_psi_der # PD Yaw Control

        # Desired Torques in body frame
        torque_phi = -self.Ix*math.sin(theta)*acc_psi
        torque_theta = self.Iy*acc_theta
        torque_psi = self.Iz*math.cos(theta)*acc_psi

        return [self.F, torque_phi, torque_theta, torque_psi]