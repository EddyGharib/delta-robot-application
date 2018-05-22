import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d,Axes3D
from math import *
from delta_kinematics import *



# Main functions
def test_3d():
    ion()
    x=[-1,1,0,-1]
    y=[-1,-1,1,-1]
    fig = plt.figure()
    ax = Axes3D(fig)
    ax.plot3D(x, y)
    plt.show(block = True)

def draw_delta_robot(theta1, theta2, theta3, ax, drob_dimensions = default_drob_dimensions):
    """
    :param theta1: First characteristic angle of the robot in degrees
    :param theta2: Second characteristic angle of the robot in degrees
    :param theta3: Third characteristic angle of the robot in degrees
    :param ax: The axis on which the robot should be plotted
    :return: draws the robot in the corresponding configuration on the given axis
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

    [x0, y0, z0] = fwd_kinematics(theta1, theta2, theta3, drob_dimensions)

    dtr = pi/180

    #Transform the angles to radian
    theta1 *= dtr
    theta2 *= dtr
    theta3 *= dtr

    #coordinated of the fixed part corners
    P1 = [-sqrt(3)*R, -R, 0]
    P2 = [sqrt(3)*R, -R, 0]
    P3 = [0, 2*R, 0]

    #coordinates of the junctions between the hands and the fixed part
    A1 = [0, -R, 0]
    A2 = [sqrt(3)/2*R, 0.5*R, 0]
    A3 = [-sqrt(3)/2*R, 0.5*R, 0]

    #coordinates of the intersection between the hands and the parallel bars
    B1 = [0, -R-r*cos(theta1), -r*sin(theta1)]
    B2 = [sqrt(3)/2*(R+r*cos(theta2)), 0.5*(R+r*cos(theta2)), -r*sin(theta2)]
    B3 = [-sqrt(3)/2*(R+r*cos(theta3)), 0.5*(R+r*cos(theta3)), -r*sin(theta3)]

    #coordinates of the intersection between the moving part and the parallel bars
    C1 = [x0, y0-d, z0]
    C2 = [x0+sqrt(3)/2*d, y0+0.5*d, z0]
    C3 = [x0-sqrt(3)/2*d, y0+0.5*d, z0]

    #coordinates of the corners of the moving part
    Q1 = [x0-sqrt(3)*d, y0-d, z0]
    Q2 = [x0+sqrt(3)*d, y0-d, z0]
    Q3 = [x0, y0+2*d, z0]

    #Creation of the triangles
    x_base_tri = [P1[0],P2[0],P3[0],P1[0]]
    y_base_tri = [P1[1],P2[1],P3[1],P1[1]]
    z_base_tri = [P1[2],P2[2],P3[2],P1[2]]
    x_end_tri = [Q1[0],Q2[0],Q3[0],Q1[0]]
    y_end_tri = [Q1[1],Q2[1],Q3[1],Q1[1]]
    z_end_tri = [Q1[2],Q2[2],Q3[2],Q1[2]]

    #Creation of the Arms and forearms
    x_arm_forearm1 = [A1[0],B1[0],C1[0]]
    y_arm_forearm1 = [A1[1],B1[1],C1[1]]
    z_arm_forearm1 = [A1[2],B1[2],C1[2]]
    x_arm_forearm2 = [A2[0],B2[0],C2[0]]
    y_arm_forearm2 = [A2[1],B2[1],C2[1]]
    z_arm_forearm2 = [A2[2],B2[2],C2[2]]
    x_arm_forearm3 = [A3[0],B3[0],C3[0]]
    y_arm_forearm3 = [A3[1],B3[1],C3[1]]
    z_arm_forearm3 = [A3[2],B3[2],C3[2]]

    ax.cla()
    ax.set_xlabel("X(m)")
    ax.set_ylabel("Y(m)")
    ax.set_zlabel("Z(m)")
    ax.set_zlim(-r-l,0.05)
    ax.set_ylim(-R-r,R+r)
    ax.set_xlim(1.1*(-R-r),1.1*(R+r))
    #Draw the triangles
    ax.plot3D(x_base_tri, y_base_tri, z_base_tri)
    ax.plot3D(x_end_tri, y_end_tri, z_end_tri)

    #Draw the arms and forearms
    ax.plot3D(x_arm_forearm1, y_arm_forearm1, z_arm_forearm1)
    ax.plot3D(x_arm_forearm2, y_arm_forearm2, z_arm_forearm2)
    ax.plot3D(x_arm_forearm3, y_arm_forearm3, z_arm_forearm3)

    plt.draw()

def animate_robot():
    #Phase 1
    z = np.arange(-0.9,-0.7,0.01)
    x = -0.3*np.ones(np.shape(z))
    y = np.zeros(np.shape(z))

    #Phase 2
    AJ1 = np.arange(-0.3,0.3,0.01)
    AJ10 = np.zeros(np.shape(AJ1))
    AJ107 = -0.7*np.ones(np.shape(AJ1))
    x = np.append(x,AJ1)
    z = np.append(z,AJ107)
    y = np.append(y,AJ10)

    #Phase 3
    AJ2 = np.arange(-0.7,-0.9,-0.01)
    AJ20 = np.zeros(np.shape(AJ2))
    AJ203 = 0.3*np.ones(np.shape(AJ2))
    x = np.append(x, AJ203)
    z = np.append(z, AJ2)
    y = np.append(y, AJ20)

    plt.ion()
    plt.hold(True)
    fig = plt.figure()
    ax = Axes3D(fig)


    for i in range(len(x)):
        # plt.clf()
        [teta1, teta2, teta3] = inv_kinematics(x[i], y[i], z[i])
        draw_delta_robot(teta1, teta2, teta3, ax)
        # time.sleep(1)
        plt.pause(0.001)
    plt.show(block=True)

def test_drawRob():
    plt.ion()
    # Initialization
    fig = plt.figure()
    ax = Axes3D(fig)
    draw_delta_robot(44.99, 44.99, 44.99, ax)
    plt.pause(2)
    draw_delta_robot(22, 22, 22, ax)
    plt.show(block=True)

def ax_plot_cube(ax, points, color = 'y'):
    ax.plot3D([points[0][0],points[1][0]], [points[0][1],points[1][1]], [points[0][2],points[1][2]], color)
    ax.plot3D([points[1][0],points[2][0]], [points[1][1],points[2][1]], [points[1][2],points[2][2]], color)
    ax.plot3D([points[2][0],points[3][0]], [points[2][1],points[3][1]], [points[2][2],points[3][2]], color)
    ax.plot3D([points[0][0],points[3][0]], [points[0][1],points[3][1]], [points[0][2],points[3][2]], color)
    ax.plot3D([points[4][0],points[5][0]], [points[4][1],points[5][1]], [points[4][2],points[5][2]], color)
    ax.plot3D([points[5][0],points[6][0]], [points[5][1],points[6][1]], [points[5][2],points[6][2]], color)
    ax.plot3D([points[6][0],points[7][0]], [points[6][1],points[7][1]], [points[6][2],points[7][2]], color)
    ax.plot3D([points[4][0],points[7][0]], [points[4][1],points[7][1]], [points[4][2],points[7][2]], color)
    ax.plot3D([points[0][0],points[4][0]], [points[0][1],points[4][1]], [points[0][2],points[4][2]], color)
    ax.plot3D([points[1][0],points[5][0]], [points[1][1],points[5][1]], [points[1][2],points[5][2]], color)
    ax.plot3D([points[2][0],points[6][0]], [points[2][1],points[6][1]], [points[2][2],points[6][2]], color)
    ax.plot3D([points[3][0],points[7][0]], [points[3][1],points[7][1]], [points[3][2],points[7][2]], color)
# Test and plots
# test_3d()

# test_drawRob()
