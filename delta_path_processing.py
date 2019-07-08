"""
Analysing the path taken by the delta robot
"""

from delta_kinematics import *
import numpy as np
import matplotlib.pyplot as plt

def dummy_go(initial_pos, final_pos, speed, disc_points=50):
    """
    :param initial_pos: Initial position of the robot
    :param final_pos: Destination of the robot
    :param disc_points: Number of discretisation points
    :return: [x, y, z] 3 vectors representing the path that the robot would take if it was controlled
    the dummy way
    """
    x_i, y_i, z_i = initial_pos
    x_f, y_f, z_f = final_pos
    res = inv_kinematics(x_i, y_i, z_i)
    res1 = inv_kinematics(x_f, y_f, z_f)
    if res == False or res1 == False:
        return False
    [t1_i, t2_i, t3_i] = res
    [t1_f, t2_f, t3_f] = res1

    #calculating the senses of the angles
    t1_s = t2_s = t3_s = 1
    if t1_f < t1_i:
        t1_s = -1
    if t2_f < t2_i:
        t2_s = -1
    if t3_f < t3_i:
        t3_s = -1

    #Create the time vector
    time1 = abs(t1_f - t1_i )/ speed
    time2 = abs(t2_f - t2_i )/ speed
    time3 = abs(t3_f - t3_i )/ speed
    tm = max(time1, time2, time3)
    t_vect = np.linspace(0, tm, disc_points)

    #Fill theta vectors
    t1_vect = np.array([t1_i])
    t2_vect = np.array([t2_i])
    t3_vect = np.array([t3_i])
    for i in range(1,disc_points):
        if t_vect[i] < time1:
            t_add = t1_i + t1_s*speed*t_vect[i]
            t1_vect = np.append(t1_vect, t_add)
        else:
            t1_vect = np.append(t1_vect, t1_f)
        if t_vect[i] < time2:
            t_add = t2_i + t2_s*speed*t_vect[i]
            t2_vect = np.append(t2_vect, t_add)
        else:
            t2_vect = np.append(t2_vect, t2_f)
        if t_vect[i] < time3:
            t_add = t3_i + t3_s*speed*t_vect[i]
            t3_vect = np.append(t3_vect, t_add)
        else:
            t3_vect = np.append(t3_vect, t3_f)

    #Fill x,,y,z vectors
    x_vect = y_vect = z_vect = np.array([])
    for i in range(disc_points):
        res = fwd_kinematics(t1_vect[i], t2_vect[i], t3_vect[i])
        if res == False:
            return "path passing by unreachable points"
        x_vect = np.append(x_vect,res[0])
        y_vect = np.append(y_vect,res[1])
        z_vect = np.append(z_vect,res[2])
    return [t_vect, [x_vect, y_vect, z_vect], [t1_vect, t2_vect, t3_vect]]

def test_dummy_go():
    """
    :return: Tests the dummy_go function
    """
    init_pos = [0, 0.2, -0.79]
    final_pos = [0, -0.2, -0.79]
    [t, [x_vect, y_vect, z_vect], [t1_vect, t2_vect, t3_vect]] = dummy_go(init_pos, final_pos, 10, 100)

    plt.hold(True)

    plt.subplot(2, 1, 1)
    plt.plot(t, t1_vect)
    plt.plot(t, t2_vect)
    plt.plot(t, t3_vect, ".")
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

if __name__ == "__main__":
    # test_dummy_go()
    pass