#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

# FK

## Symbols
q1, q2, q3, q4, q5, q6, q6, q7 = symbols('q1:9')
d1, d2, d3, d4, d5, d6, d6, d7 = symbols('d1:9')
a0, a1, a2, a3, a4, a5, a6     = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')


## DH Parameters
s = {
    alpha0:     0, a0:      0, d1:   0.75,
    alpha1: -pi/2, a1:   0.35, d2:      0, q2: q2-pi/2,
    alpha2:     0, a2:   1.25, d3:      0,
    alpha3: -pi/2, a3:    1.5, d4: -0.054,
    alpha4:  pi/2, a4:      0, d5:      0,
    alpha5: -pi/2, a5:      0, d6:      0,
    alpha6:     0, a6:      0, d7:  0.303, q7: 0
}

# ??

s = {
    alpha0:     0, a0:      0, d1:   0.75,
    alpha1: -pi/2, a1:   0.35, d2:      0, q2: q2-pi/2,
    alpha2:     0, a2:   1.25, d3:      0,
    alpha3: -pi/2, a3: -0.054, d4:    1.5, 
    alpha4:  pi/2, a4:      0, d5:      0,
    alpha5: -pi/2, a5:      0, d6:      0,
    alpha6:     0, a6:      0, d7:  0.303, q7: 0
}



R0_3 = Matrix([
[sin(q2 + q3)*cos(q1), cos(q1)*cos(q2 + q3), -sin(q1)],
[sin(q1)*sin(q2 + q3), sin(q1)*cos(q2 + q3),  cos(q1)],
[        cos(q2 + q3),        -sin(q2 + q3),        0]])


## Correction to account orientation difference between definition of 
## gripper_link in URDF vs DH Convention
R_z = Matrix([
    [cos(pi), -sin(pi), 0],
    [sin(pi),  cos(pi), 0],
    [      0,        0, 1],
])
R_y = Matrix([
    [ cos(-pi/2),  0, sin(-pi/2)],
    [          0,  1,          0],
    [-sin(-pi/2),  0, cos(-pi/2)],
])
R_corr = simplify(R_z * R_y)

def get_Rz(angle): 
    return Matrix([
        [cos(angle), -sin(angle), 0, ],
        [sin(angle),  cos(angle), 0, ],
        [         0,           0, 1, ],
    ])

def get_Ry(angle): 
    return Matrix([
        [ cos(angle),  0, sin(angle), ],
        [          0,  1,          0, ],
        [-sin(angle),  0, cos(angle), ],
    ])

def get_Rx(angle): 
    return Matrix([
        [1,          0,            0, ],
        [0, cos(angle),  -sin(angle), ],
        [0, sin(angle),   cos(angle), ],
    ])

def limit_value(val):
    if val > 1:
        return 1
    if val < -1:
        return -1
    return val

def handle_calculate_IK(req):
    
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1

    # Initialize service response
    joint_trajectory_list = []
    for x in xrange(0, len(req.poses)):
        
        p_x = req.poses[x].position.x
        p_y = req.poses[x].position.y
        p_z = req.poses[x].position.z

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([
            req.poses[x].orientation.x, 
            req.poses[x].orientation.y,
            req.poses[x].orientation.z, 
            req.poses[x].orientation.w,
        ])

        ## IK
        T_rpy = get_Rz(yaw) * get_Ry(pitch) * get_Rx(roll) * R_corr
        R_rpy = Matrix(T_rpy[0:3,0:3])

        n_x = R_rpy[0, 2]
        n_y = R_rpy[1, 2]
        n_z = R_rpy[2, 2]

        R_rpy = get_Rz(yaw) * get_Ry(pitch) * get_Rx(roll) * R_corr
        n_x, n_y, n_z = R_rpy[0, 2], R_rpy[1, 2], R_rpy[2, 2]
        
        w_x = float(p_x - (s[d6] + s[d7]) * n_x)
        w_y = float(p_y - (s[d6] + s[d7]) * n_y)
        w_z = float(p_z - (s[d6] + s[d7]) * n_z)
        
        #    
        # Angles
        # 
        
        # Theta 1
        theta1 = atan2(w_y, w_x)

        # Theta 2
        _A = 1.5
        _C = s[a2]
        _B = sqrt(w_x**2 + w_y**2 + (w_z-s[d1])**2)
        _alpha = acos((_A**2 - _C**2 - _B**2) / (-2*_A*_B))
        _D = sqrt(w_x**2 + w_y**2)
        _K = w_z - s[d1]
        _beta = acos(_D / _B)

        theta2 = float(pi/2 - _alpha - _beta)

        # Theta 3
        _gamma = acos((_B**2-_C**2-_A**2)/(-2*_A*_C))
        theta3 = pi/2 - _gamma - atan2(0.0054, 1.5)

        # Theta 4
        R0_3_inv  = R0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3}).inv(method="LU") 
        R3_6   = R0_3_inv * R_rpy

        theta4 = atan2(R3_6[2,2], -R3_6[0,2])
        
        # Theta 5
        theta5 = atan2(sqrt(R3_6[0,2]**2+R3_6[2,2]**2), R3_6[1,2])
        
        # Theta 6
        theta6 = atan2(-R3_6[1,1], R3_6[1,0])


        joint_trajectory_point = JointTrajectoryPoint()
        joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
        joint_trajectory_point.positions = [float(p) for p in joint_trajectory_point.positions]
        joint_trajectory_list.append(joint_trajectory_point)

    rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
    return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
