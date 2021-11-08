#!/usr/bin/env python

import rospy
import numpy as np
import math

def calculate_allocation_matrix(d, Ct, Cm):
    # Calculate matrix that maps the rotor veocities with the torques and thrust, allowing to consider them as our inputs

    reaction_thrust = d/np.sqrt(2)*Ct
    A = np.array([[Ct, Ct, Ct, Ct],
                [-reaction_thrust, -reaction_thrust, reaction_thrust, reaction_thrust],
                [-reaction_thrust, reaction_thrust, reaction_thrust, -reaction_thrust],
                [-Cm*Ct, Cm*Ct, -Cm*Ct, Cm*Ct]]) # Allocation Matrix, last line SPECIFIC to rotor_S implementation

    A_inv = np.linalg.inv(A) # Inverse of the allocation matrix
    return A_inv

def angular_diff(ang1, ang2):
    # Returns angular difference wrapped to [-pi, pi]

    error = ang2 - ang1
    sin = math.sin(error)
    cos = math.cos(error)

    ang_diff = math.atan2(sin, cos)
    return ang_diff

def saturate_value(var, max_value=None, min_value= None):

    if max_value != None and var > max_value:
        var = max_value
    if min_value != None and var < min_value:
        var = min_value

    return var

def calculate_rotor_velocities(cmd_torques, A_inv, omega_e, mode):
    # Calculate the required rotor velocities to achieve desired torques and thrust 

    rotor_velocities_squared = np.dot(A_inv, cmd_torques) # Get the squared angular velocites (A.wi^2 = Tau)

    # Making sure the rotor velocities are not negative and do not cause the robot to take-off
    for i, vel in enumerate(rotor_velocities_squared):
        if mode == 'ground' and vel > omega_e:
            rotor_velocities_squared[i] = omega_e
        if vel < 0:
            rotor_velocities_squared[i] = 0

    rotor_velocities = np.sqrt(rotor_velocities_squared)

    #rospy.loginfo('Commanded Rotor Velocities: %s', rotor_velocities)
    return rotor_velocities

def land_robot(linear_velocity, A_inv, landed, t_landed, force):
    # Checks wether the robot has landed on an inclined surface or on flat ground

    cmd_torque = [force*0.95, 0, 0, 0]
    rotor_velocities = calculate_rotor_velocities(cmd_torque, A_inv, 0, 'land')

    vx = linear_velocity[0]
    vz = linear_velocity[2]

    if landed == True: 
        if t_landed < 1:
            if abs(vx) > 0.01 and vz < -0.1:
                mode = 'inclined'
                if vx > 0:
                    gamma = math.atan2(vz, vx)
                else:
                    gamma = math.atan2(vz, vx) + math.pi
                rospy.loginfo('Detected an inclined surface with inclination: %f. Turning on Inclined Controller', gamma)
            else:
                mode = 'land'
                gamma = 0
        else:
            mode = 'ground'
            gamma = 0
            rospy.loginfo('Detected a flat surface, turning on Ground Controler')

    else:
        mode = 'land'
        gamma = 0

    return [rotor_velocities, mode, gamma]

def print_gamma(gamma):
    # Print Gamma in inclination.txt

	gamma_out = open('inclination.txt','w+')
	gamma_out.write('Gamma:'+ str(gamma))
	gamma_out.close()