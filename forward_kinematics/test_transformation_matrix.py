import numpy as np

T = np.array([
[0.887543656363152,   0.164020283806259,  0.430538737571477, 2.28643969919972],
[0.149063166495295,  -0.986451215812157, 0.0685140220474088, 0.30016154739919],
[0.435943150472539, 0.00336828188118831, -0.899967901779417,  2.7341899707775],
[                0,                   0,                  0,              1.0]])

origin = np.array([0, 0, 0, 1]).T

print(origin * T)