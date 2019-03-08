## Project: Kinematics Pick & Place

**Sergei Surovtsev**
<br/>
Udacity Robotics Software Engineer Nanodegree
<br/>
Class of November 2018

## Project Description

This project is an introduction to industrial robotics. In involves controlling a robotic arm and modelling it's forward and backward kinematics. The problem we are solving is control an industrial arm to grab an object from shelf and drop it in a bin.

Robotic arm used in this project is [KR210](https://www.youtube.com/watch?v=HudUt4MYpyc).

## Project Goals

* Introduction to Robot Operating System (ROS) development and debugging
* Intruduction to real-world industrial robotics
* Introduction to robot kinematics

## Technical Formulation of Problem 

* Set up environment as described in [Project Repository](https://github.com/udacity/RoboND-Kinematics-Project)
* Perform forward kinematics analysis
* Derive angles for inverse kinematics
* Write ROS node for solving IK in kuka_arm/scripts/IK_server.py

## Mathematical Models

### Forward Kinematics

Problem of Forward Kinematics (FK) is finding position and orientation of end-effector of robotic arm given it's joint angles. 

To solve FK we need to compose a DH-parameter table which describes robot's physical parameters such as distance between joints and angle transforms.

```python
s = {
    alpha0:     0, a0:      0, d1:   0.75,
    alpha1: -pi/2, a1:   0.35, d2:      0, q2: q2-pi/2,
    alpha2:     0, a2:   1.25, d3:      0,
    alpha3: -pi/2, a3: -0.054, d4:    1.5, 
    alpha4:  pi/2, a4:      0, d5:      0,
    alpha5: -pi/2, a5:      0, d6:      0,
    alpha6:     0, a6:      0, d7:  0.303, q7: 0
}
```

### Inverse Kinematics Kinematics

Problem of Inverse Kinematics (IK) is defined as finding joint angles given position and orientation of end-effector. This can be a problem with large solution space but for robot model in our case there is a known fast analitical method of solving it. 

I've used method suggested in lectures to derive angles.

```python
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
```

### Debugging

Refer to IK_debug.py for forward and backward kinematics test cases.

### Results

KR210 is able to perform a task at hand. [video](https://www.youtube.com/watch?v=PC-fSU6Bn2A&feature=youtu.be)
