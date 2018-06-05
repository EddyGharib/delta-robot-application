"""
This python scripts handles the workspace analysis in 2D using a smart algorithm.
Points in 2D are interpreted as numpy complex numbers like 1+2j.
The algorithm starts by considering 1 internal point and one external point of the workspace.
Then using binary search, these 2 points are moved to a close position of the boundaries of the workspace.
At this point the starting point to be considered is the center of these 2 close point, and the direction
    from this point is pi/2 + the direction of the vector (outside_pt, inside_pt).
From the position of this point we consider the arc of radius R, value 180deg and where the direction 90deg is
    the same as the direction considered in the starting point.
Then this arc is divided to 2^(k) sectors, each sector of value pi/(2^k) radians.
Using binary search, the sector that would limit the workspace (having 1 side in the workspace and the other outside it
    would be considered to calculate the next point and the next direction.
"""

from delta_kinematics import *
from mpl_toolkits.mplot3d import axes3d, Axes3D
from math import *

def get_close_pts(pt_in, pt_out, z, eps):
    pt1 = pt_in
    pt2 = pt_out
    while(np.linalg.norm(pt1-pt2)>eps):
        middle = (pt1+pt2)/2
        res = inv_kinematics(middle.real, middle.imag, z)
        if res == False:
            pt2 = middle
        else:
            pt1 = middle
    return pt1, pt2

def get_next_pt_and_direction(pt, direction, z, R):
    pt1_angle = direction - pi/2
    pt2_angle = direction + pi/2
    pt1 = pt + R*(cos(pt1_angle)+1j*sin(pt1_angle))
    pt2 = pt + R*(cos(pt2_angle)+1j*sin(pt2_angle))

    for i in range(8):
        middle_angle = (pt1_angle+pt2_angle)/2
        middle_pt = pt + R*(cos(middle_angle) + 1j*sin(middle_angle))
        res = inv_kinematics(middle_pt.real, middle_pt.imag, z)
        if res == False:
            pt2 = middle_pt
            pt2_angle = middle_angle
        else:
            pt1 = middle_pt
            pt1_angle = middle_angle
    ret_pt = (pt1+pt2)/2
    ret_angle = np.angle(pt1 - pt2) + pi/2
    return ret_pt, ret_angle

def get_bound_pts(pt_in, pt_out, z, R, eps_initial=0.01):
    pts = []
    pt1, pt2 = get_close_pts(pt_in, pt_out,z,eps_initial)
    start_pt = (pt1 + pt2)/2
    pts.append(start_pt)
    print(pt1)
    print(pt2)
    print(np.angle(pt1-pt2) + pi/2)
    next_pt, next_dir = get_next_pt_and_direction(start_pt, np.angle(pt1-pt2) + pi/2, z, R)
    pts.append(next_pt)
    while(np.linalg.norm(next_pt-start_pt) > 0.99*R):
        next_pt, next_dir = get_next_pt_and_direction(next_pt, next_dir, z, R)
        pts.append(next_pt)
    return pts

def split_complex_array(arr):
    arr = np.array(arr)
    return arr.real,arr.imag



"""testing get_close_pts"""
# pt1,pt2 = get_close_pts(0, 1+1j, -0.7, 0.01)
# # print(pt1, pt2)


"""Testing get_bound_pts"""
# pts = get_bound_pts(0, 1, -0.3, 0.01)
# # x, y = split_complex_array(pts)
# # plot(x, y, 'r.')
# # show()
