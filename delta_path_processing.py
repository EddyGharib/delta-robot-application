"""
Analysing the path taken by the delta robot
"""

from delta_kinematics import *
import numpy as np
import matplotlib.pyplot as plt

def dummy_go(initial_pos, final_pos, disc_points=50):
    """
    :param initial_pos: Initial position of the robot
    :param final_pos: Destination of the robot
    :param disc_points: Number of discretisation points
    :return: [x, y, z] 3 vectors representing the path that the robot would take if it was controlled
    the dummy way
    """
    x_i = initial_pos[0]
    y_i = initial_pos[1]
    z_i = initial_pos[2]
    x_f = final_pos[0]
    y_f = final_pos[1]
    z_f = final_pos[2]
    res = inv_kinematics(x_i, y_i, z_i)
    res1 = inv_kinematics(x_f, y_f, z_f)
    if res == "position does not exist" or res1 == "position does not exist":
        return "position does not exist"
    [t1_i, t2_i, t3_i] = res
    [t1_f, t2_f, t3_f] = res1
    t1_vect = np.linspace(t1_i, t1_f, disc_points)
    t2_vect = np.linspace(t2_i, t2_f, disc_points)
    t3_vect = np.linspace(t3_i, t3_f, disc_points)
    x_vect = y_vect = z_vect = np.array([])
    for i in range(disc_points):
        res = fwd_kinematics(t1_vect[i], t2_vect[i], t3_vect[i])
        if res == "position does not exist":
            return "path passing by unreachable points"
        x_vect = np.append(x_vect,res[0])
        y_vect = np.append(y_vect,res[1])
        z_vect = np.append(z_vect,res[2])
    return [x_vect, y_vect, z_vect]

def test_dummy_go():
    """
    :return: Tests the dummy_go function
    """
    init_pos = [0, -0.18, -0.79]
    final_pos = [0, 0.2, -0.79]
    init_tetas = inv_kinematics(init_pos[0], init_pos[1], init_pos[2])
    final_tetas = inv_kinematics(final_pos[0], final_pos[1], final_pos[2])
    t1_vect = np.linspace(init_tetas[0], final_tetas[0], 50)
    t2_vect = np.linspace(init_tetas[1], final_tetas[1], 50)
    t3_vect = np.linspace(init_tetas[2], final_tetas[2], 50)
    [x_vect, y_vect, z_vect] = dummy_go(init_pos, final_pos)

    t = np.linspace(0, 1, 50)

    plt.hold(True)

    plt.subplot(2, 1, 1)
    plt.plot(t, t1_vect)
    plt.plot(t, t2_vect)
    plt.plot(t, t3_vect)
    plt.title("Theta angle variations")
    plt.legend(["T1", "T2", "T3"])
    plt.ylabel("theta(degree)")

    plt.subplot(212)
    plt.plot(t,x_vect)
    plt.plot(t,y_vect)
    plt.plot(t,z_vect)
    plt.title("Position of the end effector")
    plt.legend(["x", "y", "z"])
    plt.xlabel("time(s)")
    plt.ylabel("position(m)")
    plt.show()

    plt.figure()
    plt.plot(t, z_vect)
    plt.xlabel("time(s)")
    plt.ylabel("Z axis position (m)")
    plt.legend("Z position")
    plt.title("Z axis position")
    plt.show()
test_dummy_go()
