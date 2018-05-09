"""
Python implementation of the direct (forward) and inverse kinematic equations
of a delta robot with special dimensions. R, r, l and d.
"""

import numpy as np
from math import *
import time
from matplotlib.pyplot import *



# Some other constants
sqrt3 = sqrt(3)
sin120 = sqrt(3) / 2
cos120 = -0.5
tan60 = sqrt(3)
sin30 = 0.5
tan30 = 1 / sqrt(3)

default_drob_dimensions = {'R':0.40941, 'r':0.4, 'l':0.8, 'd':0.07845}
c_default_drob_dimensions = {'R':0.40941, 'r':0.4, 'l':0.8, 'd':0.07845}
# Main functions
def fwd_kinematics(theta1, theta2, theta3, drob_dimensions = default_drob_dimensions):
    """
    Takes the angles in degrees and gives the position if existing
    else it returns "position does not exist"
    """
    # Robot dimensions
    d = drob_dimensions['d']
    R = drob_dimensions['R']
    r = drob_dimensions['r']
    l = drob_dimensions['l']

    e = 2 * sqrt(3) * d
    f = 2 * sqrt(3) * R
    re = l
    rf = r

    t = (f - e) * tan30 / 2
    dtr = pi / 180

    theta1 *= dtr
    theta2 *= dtr
    theta3 *= dtr

    y1 = -(t + rf * cos(theta1))
    z1 = -rf * sin(theta1)

    y2 = (t + rf * cos(theta2)) * sin30
    x2 = y2 * tan60
    z2 = -rf * sin(theta2)

    y3 = (t + rf * cos(theta3)) * sin30
    x3 = -y3 * tan60
    z3 = -rf * sin(theta3)

    dnm = (y2 - y1) * x3 - (y3 - y1) * x2

    w1 = y1 * y1 + z1 * z1
    w2 = x2 * x2 + y2 * y2 + z2 * z2
    w3 = x3 * x3 + y3 * y3 + z3 * z3

    a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
    b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0

    a2 = -(z2 - z1) * x3 + (z3 - z1) * x2
    b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0

    a = a1 * a1 + a2 * a2 + dnm * dnm
    b = 2 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm)
    c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - re * re)

    d = b * b - 4.0 * a * c;

    if (d < 0):
        return "position does not exist"

    z0 = -0.5 * (b + sqrt(d)) / a;
    x0 = (a1 * z0 + b1) / dnm
    y0 = (a2 * z0 + b2) / dnm

    return [x0, y0, z0]


def s_inv_kinematics(x0, y0, z0, drob_dimensions = default_drob_dimensions):
    """
    Takes the position of the end effector as input and returns
    the corresponding angle in degree if existing
    else it returns "position does not exist"
    """

    # Robot dimensions
    d = drob_dimensions['d']
    R = drob_dimensions['R']
    r = drob_dimensions['r']
    l = drob_dimensions['l']

    e = 2 * sqrt(3) * d
    f = 2 * sqrt(3) * R
    re = l
    rf = r

    y1 = -0.5 * 0.57735 * f
    y0 -= 0.5 * 0.57735 * e

    a = (x0 * x0 + y0 * y0 + z0 * z0 + rf * rf - re * re - y1 * y1) / (2 * z0)
    b = (y1 - y0) / z0

    d = -(a + b * y1) * (a + b * y1) + rf * (b * b * rf + rf)
    if (d < 0):
        return "position does not exist"

    yj = (y1 - a * b - sqrt(d)) / (b * b + 1)
    zj = a + b * yj

    if (yj > y1):
        toadd = 180
    else:
        toadd = 0
    theta = 180 * atan(-zj / (y1 - yj)) / pi + toadd
    return theta


def inv_kinematics(x0, y0, z0, drob_dimensions = default_drob_dimensions):
    """
    Takes the position of the end effector as input and returns
    the 3 angles in degree if existing
    else it returns "position does not exist"
    """

    # Robot dimensions
    d = drob_dimensions['d']
    R = drob_dimensions['R']
    r = drob_dimensions['r']
    l = drob_dimensions['l']

    e = 2 * sqrt(3) * d
    f = 2 * sqrt(3) * R
    re = l
    rf = r

    theta1 = s_inv_kinematics(x0, y0, z0, drob_dimensions)
    theta2 = s_inv_kinematics(x0 * cos120 + y0 * sin120, y0 * cos120 - x0 * sin120, z0, drob_dimensions)
    theta3 = s_inv_kinematics(x0 * cos120 - y0 * sin120, y0 * cos120 + x0 * sin120, z0, drob_dimensions)
    if (
            theta1 == "position does not exist" or theta2 == "position does not exist" or theta3 == "position does not exist"):
        return "position does not exist"
    return [theta1, theta2, theta3]


def mesure_inv_kin_speed():
    start_time = time.clock()
    inv_kinematics(0, 0, -0.941)
    print("--- %s seconds ---" % (time.clock() - start_time))


# mesure_inv_kin_speed()


# print(inv_kinematics(0.5, 0.5, -0.8))
