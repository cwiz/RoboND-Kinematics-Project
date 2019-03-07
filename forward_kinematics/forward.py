
import numpy as np
import sys

#from sympy import array
from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2
from sympy.matrices import Matrix

# Symbols for joint variables
q1, q2, q3, q4, q5, q6, q6, q7 = symbols('q1:9')
d1, d2, d3, d4, d5, d6, d6, d7 = symbols('d1:9')
a0, a1, a2, a3, a4, a5, a6     = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

# Kuka KR210
# DH Parameters

# alpha[i] - twist angle
# a[i]     - link lengths
# d[i]     - link offset
# q[i]     - joint variables

s = {
	alpha0:     0, a0:      0, d1:  0.75,
	alpha1: -pi/2, a1:   0.35, d2:     0, q2: q2-pi/2,
	alpha2:     0, a2:   1.25, d3:     0,
	alpha3: -pi/2, a3: -0.054, d4:  1.50,
	alpha4:  pi/2, a4:      0, d5:     0,
	alpha5: -pi/2, a5:      0, d6:     0,
	alpha6:     0, a6:      0, d7: 0.303, q7: 0
}


# Homogenous Transforms
# base_links transforms 
print("Defining matrices")

T0_1 = Matrix([
	[ cos(q1),                        -sin(q1),            0,              a0 ],
	[ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1 ],
	[ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1 ],
	[                   0,                   0,            0,               1 ],
])
T0_1 = T0_1.subs(s)

T1_2 = Matrix([
	[ cos(q2),                        -sin(q2),            0,              a1 ],
	[ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2 ],
	[ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2 ],
	[                   0,                   0,            0,               1 ],
])
T1_2 = T1_2.subs(s)

T2_3 = Matrix([
	[ cos(q3),                        -sin(q3),            0,              a2 ],
	[ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3 ],
	[ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3 ],
	[                   0,                   0,            0,               1 ],
])
T2_3 = T2_3.subs(s)

T3_4 = Matrix([
	[ cos(q4),                        -sin(q4),            0,              a3 ],
	[ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4 ],
	[ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4 ],
	[                   0,                   0,            0,               1 ],
])
T3_4 = T3_4.subs(s)

T4_5 = Matrix([
	[ cos(q5),                        -sin(q5),            0,              a4 ],
	[ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5 ],
	[ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5 ],
	[                   0,                   0,            0,               1 ],
])
T4_5 = T4_5.subs(s)

T5_6 = Matrix([
	[ cos(q6),                        -sin(q6),            0,              a5 ],
	[ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6 ],
	[ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6 ],
	[                   0,                   0,            0,               1 ],
])
T5_6 = T5_6.subs(s)

T6_G = Matrix([
	[ cos(q7),                        -sin(q7),            0,              a6 ],
	[ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7 ],
	[ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7 ],
	[                   0,                   0,            0,               1 ],
])
T6_G = T6_G.subs(s)


# Composition of homogenous transforms
print("Simplifying homogenous transforms")
T0_1 = simplify(T0_1)        # base_link
T0_2 = simplify(T0_1 * T1_2) # base_link to link_2
T0_3 = simplify(T0_2 * T2_3) # base_link to link_3
T0_4 = simplify(T0_3 * T3_4) # base_link to link_4
T0_5 = simplify(T0_4 * T4_5) # base_link to link_5
T0_6 = simplify(T0_5 * T5_6) # base_link to link_6
T0_G = simplify(T0_6 * T6_G) # base_link to link_g


# Numerically evaluate transforms 
# print("T0_1 = ", T0_1.evalf(subs=params) * origin)
# print("T0_2 = ", T0_2.evalf(subs=params) * origin)
# print("T0_3 = ", T0_3.evalf(subs=params) * origin)
# print("T0_4 = ", T0_4.evalf(subs=params) * origin)
# print("T0_5 = ", T0_5.evalf(subs=params) * origin)
# print("T0_6 = ", T0_6.evalf(subs=params) * origin)
# print("T0_G = ", T0_G.evalf(subs=params) * origin)

# Correction to account orientation difference between definition of 
# gripper_link in URDF vs DH Convention
print("Applying gripper transformation")
R_z = Matrix([
	[cos(pi), -sin(pi), 0, 0],
	[sin(pi),  cos(pi), 0, 0],
	[      0,        0, 1, 0],
	[      0,        0, 0, 1],
])
R_y = Matrix([
	[ cos(-pi/2),  0, sin(-pi/2), 0],
	[          0,  1,          0, 0],
	[-sin(-pi/2),  0, cos(-pi/2), 0],
	[          0,  0,          0, 1],
])
R_corr = simplify(R_z * R_y)

# Total homogenous transform between base_link and gripper_link
# with orientation correction
T_total = simplify(T0_G * R_corr)

# Calculating transition Matrix for given parameters
print("Calculate transition matrix for given params")
# params = {
# 	q1: 0.13,
# 	q2: 0.77,
# 	q3: -1.60,
# 	q4: 3.23,
# 	q5: -0.38,
# 	q6: -0.07,
# }

params = {
	q1: 0,
	q2: 0,
	q3: 0,
	q4: 0,
	q5: 0,
	q6: 0,
}

print("T0_1 = ",    T0_1.evalf(subs=params))
print("T0_2 = ",    T0_2.evalf(subs=params))
print("T0_3 = ",    T0_3.evalf(subs=params))
print("T0_4 = ",    T0_4.evalf(subs=params))
print("T0_5 = ",    T0_5.evalf(subs=params))
print("T0_6 = ",    T0_6.evalf(subs=params))
print("T0_G = ",    T0_G.evalf(subs=params))
print("T_total = ", T_total.evalf(subs=params))
