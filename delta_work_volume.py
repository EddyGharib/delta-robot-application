"""
Python code used to analyse the work volume of a delta robot which
dimensions, inverse and forward kinematics are defined in delta_kinematics
"""

from delta_kinematics import *
from mpl_toolkits.mplot3d import axes3d, Axes3D
from scipy.spatial import ConvexHull
from math import *

"""
This script uses a special data type "Horizontal cut"
represented by an np.array of shape (3, y_pts_count, x_pt_count) with:
    m[0,:,:] is a (n x n) matrix containing x values of the points
    m[1,:,:] is a (n x n) matrix containing y values of the points
    m[2,:,:] = is an (n x n) matrix, with each element =1 if the corresponding point (x,y) is in the work volume, else 0
this weird structure (instead of np.shape = (x, y, 3)) is used for simple visualization
of the arrays, and to iterate rapidly over elements, however any other structure can be used.

Rq: to visually understand the 3 matrix, y is given before x
    for example:
    [   
        [array containing "x_points" values]
        [array containing "x_points" values]
        .
        .
        .y_points times
        .
        .
        [array containing "x_points" values]
    ]
"""


"""Basic functions"""
def get_horiz_cut(z=-0.9, Gamma=False, Beta=False, xmin=-1, xmax=1, ymin=-1, ymax=1, h_pointnum=50, drob_dimensions = default_drob_dimensions):
    """
    :param z: Height of the horizontal plane where the cut would be made
    :param xmin: minimum x of the horizontal cut
    :param xmax: maximum x of the horizontal cut
    :param ymin: minimum y of the horizontal cut
    :param ymax: maximum y of the horizontal cut
    :param pointnum: number of points to be considered between xmin and xmax (or ymin and ymax)
    :return: "Horizontal cut" data type representing the cut
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

    final_matrix = np.zeros((3, h_pointnum, h_pointnum))  # 3*(n x m) matrix, two for x and y, and one for "reachable point"

    x_array = np.linspace(xmin, xmax, h_pointnum)  # array to fill the matrix[0] (x)
    y_array = np.linspace(ymin, ymax, h_pointnum)  # array to fill the matrix[1] (y)
    [x_mat, y_mat] = np.meshgrid(x_array, y_array)
    final_matrix[0, :, :] = x_mat
    final_matrix[1, :, :] = y_mat
    for i in range(h_pointnum):
        for j in range(h_pointnum):
            res = is_mechanically_accessible(final_matrix[0, j, i], final_matrix[1, j, i], z, Gamma, Beta)
            if res:
                final_matrix[2, j, i] = 1
            else:
                final_matrix[2, j, i] = 0
    return final_matrix

def get_horiz_cut_boundaries(horiz_cut):
    """
    takes a "Horizontal cut" type representing the cut and returns "Horizontal cut" type
    with the boundaries only.
    :param "Horizontal cut" data type of the input cut
    :return: "Horizontal cut" data type contatining the boundaries of the provided cut
    """
    matrix_copy = np.copy(horiz_cut[2, :, :])
    shape = horiz_cut.shape
    for j in range(1, shape[1] - 1):
        for i in range(1, shape[2] - 1):
            if (horiz_cut[2, j, i - 1] == 1 and horiz_cut[2, j, i + 1] == 1
                    and horiz_cut[2, j - 1, i] == 1 and horiz_cut[2, j + 1, i] == 1):
                matrix_copy[j, i] = 0
    final_result = np.copy(horiz_cut)

    final_result[2, :, :] = matrix_copy
    return final_result

def horiz_cut_to_vect(horiz_cut):
    """
    Takes a horizontal cut structure as input and gives 2 vectors containing the coordinates of the points inside the cut
    :param horiz_cut: input horizontal cut
    :return: [x, y] arrays containing point coordinates
    """
    shape = horiz_cut.shape
    x_vect = np.array([])
    y_vect = np.array([])
    for j in range(shape[1]):
        for i in range(shape[2]):
            if (horiz_cut[2, j, i] == 1):
                x_vect = np.append(x_vect, horiz_cut[0, j, i])
                y_vect = np.append(y_vect, horiz_cut[1, j, i])
    return [x_vect, y_vect]


"""Work volume"""
def get_work_volume_points(boundary=True, Gamma=False, Beta=False, xmin=-1, xmax=1, ymin=-1, ymax=1, zmin=-1.3, zmax=-0.02, h_pointnum=50, cutsnum = 40, drob_dimensions = default_drob_dimensions):
    """
    :param xmin: Start x position of the cubic volume to be tested
    :param xmax: End x position of the cubic volume to be tested
    :param ymin: Start y position of the cubic volume to be tested
    :param ymax: End y position of the cubic volume to be tested
    :param zmin: Start z position of the cubic volume to be tested
    :param zmax: End z position of the cubic volume to be tested
    :param h_pointnum: Nmber of points to be tested between xmin and max (same is used for ymin --> ymax)
    :param cutsnum: Number of hotizontal cuts to be made between zmin and zmax
    :return: vectors holding the coordinates of the points inside the work volume
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

    x_comp = np.array([])
    y_comp = np.array([])
    z_comp = np.array([])
    #iterating from the top to the bottom
    for Z in np.linspace(zmax, zmin, cutsnum):
        if boundary == False:
            [x1, y1] = horiz_cut_to_vect(get_horiz_cut(Z, Gamma, Beta, xmin, xmax, ymin, ymax, h_pointnum, drob_dimensions))
        else:
            horiz_cut = get_horiz_cut(Z, Gamma, Beta, xmin, xmax, ymin, ymax, h_pointnum)
            [x1, y1] = horiz_cut_to_vect(get_horiz_cut_boundaries(horiz_cut))
        x_comp = np.append(x_comp, x1)
        y_comp = np.append(y_comp, y1)
        z_comp = np.append(z_comp, Z * np.ones(len(x1)))
    return [x_comp, y_comp, z_comp]

def get_z_max(drob_dimensions = default_drob_dimensions):
    # Robot dimensions
    d = drob_dimensions['d']
    R = drob_dimensions['R']
    r = drob_dimensions['r']
    l = drob_dimensions['l']

    ymin = -r-R-l
    ymax = -ymin
    xmin = -l
    xmax = l

    z_min = -r-l
    z_max = 0
    for i in range(10):
        z_moy = (z_max+z_min)/2
        h_cut = get_horiz_cut(z_moy,False,False, xmin, xmax, ymin, ymax, 50, drob_dimensions)
        if (h_cut[2,:,:].__contains__(1)):
            z_min = z_moy
        else:
            z_max = z_moy
    return z_moy


"""Mechanical limitations"""
def is_mechanically_accessible(x, y, z, Gamma = False, Beta = False, drob_dimensions = default_drob_dimensions):
    """
    :param x: X axis position of the point to be tested
    :param y: Y axis position of the point to be tested
    :param z: Z axis position of the point to be tested
    :return: True if the position is mechanically accessible, false if not
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

    res = inv_kinematics(x, y, z)
    if res == "position does not exist":
        return False
    [theta1, theta2, theta3] = res
    theta1_rd = theta1*pi/180
    theta2_rd = theta2*pi/180
    theta3_rd = theta3*pi/180

    #coordinates of the junctions between the hands and the fixed part
    A1 = np.array([0, -R, 0])
    A2 = np.array([sqrt(3)/2*R, 0.5*R, 0])
    A3 = np.array([-sqrt(3)/2*R, 0.5*R, 0])

    #coordinates of the intersection between the hands and the parallel bars
    B1 = np.array([0, -R-r*cos(theta1_rd), -r*sin(theta1_rd)])
    B2 = np.array([sqrt(3)/2*(R+r*cos(theta2_rd)), 0.5*(R+r*cos(theta2_rd)), -r*sin(theta2_rd)])
    B3 = np.array([-sqrt(3)/2*(R+r*cos(theta3_rd)), 0.5*(R+r*cos(theta3_rd)), -r*sin(theta3_rd)])

    #coordinates of the intersection between the moving part and the parallel bars
    C1 = np.array([x, y-d, z])
    C2 = np.array([x+sqrt(3)/2*d, y+0.5*d, z])
    C3 = np.array([x-sqrt(3)/2*d, y+0.5*d, z])

    if Gamma == True:
        #gamma1
        Cpr1 = np.array([0, y-d, z])
        B1C1 = C1 - B1
        B1Cpr1 = Cpr1 - B1
        gamma1_rd = acos(np.dot(B1C1,B1Cpr1)/(np.linalg.norm(B1Cpr1)*np.linalg.norm(B1C1)))
        gamma1 = gamma1_rd*180/pi

        #gamma2
        B2_120 = np.array([B2[0]*cos120+B2[1]*sin120, -B2[0]*sin120+B2[1]*cos120, B2[2]])
        C2_120 = np.array([C2[0]*cos120+C2[1]*sin120, -C2[0]*sin120+C2[1]*cos120, C2[2]])
        Cpr2_120 = np.array([0, -C2[0]*sin120+C2[1]*cos120, C2[2]])
        B2C2_120 = C2_120 - B2_120
        B2Cpr2_120 = Cpr2_120 - B2_120
        gamma2_rd = acos(np.dot(B2C2_120,B2Cpr2_120)/(np.linalg.norm(B2Cpr2_120)*np.linalg.norm(B2C2_120)))
        gamma2 = gamma2_rd*180/pi

        #gamma3
        B3_240 = np.array([B3[0]*cos120-B3[1]*sin120, B3[0]*sin120+B3[1]*cos120, B3[2]])
        C3_240 = np.array([C3[0]*cos120-C3[1]*sin120, C3[0]*sin120+C3[1]*cos120, C3[2]])
        Cpr3_240 = np.array([0, C3[0]*sin120+C3[1]*cos120, C3[2]])
        B3C3_240 = C3_240 - B3_240
        B3Cpr3_240 = Cpr3_240 - B3_240
        gamma3_rd = acos(np.dot(B3C3_240,B3Cpr3_240)/(np.linalg.norm(B3Cpr3_240)*np.linalg.norm(B3C3_240)))
        gamma3 = gamma3_rd*180/pi

        gamma_max = 49.267

        is_accessible_gamma = not (gamma1>gamma_max or gamma2>gamma_max or gamma3>gamma_max)
    if Beta == True:
        #Beta1
        Cpr1 = np.array([0, y-d, z])
        B1A1 = A1 - B1
        B1Cpr1 = Cpr1 - B1
        beta1_rd = acos(np.dot(B1A1,B1Cpr1)/(np.linalg.norm(B1Cpr1)*np.linalg.norm(B1A1)))
        beta1 = beta1_rd*180/pi

        #Beta2
        B2_120 = np.array([B2[0]*cos120+B2[1]*sin120, -B2[0]*sin120+B2[1]*cos120, B2[2]])
        Cpr2_120 = np.array([0, -C2[0]*sin120+C2[1]*cos120, C2[2]])
        A2_120 = np.array([A2[0]*cos120+A2[1]*sin120, -A2[0]*sin120+A2[1]*cos120, A2[2]])
        B2Cpr2_120 = Cpr2_120 - B2_120
        B2A2_120 = A2_120 - B2_120
        beta2_rd = acos(np.dot(B2A2_120,B2Cpr2_120)/(np.linalg.norm(B2Cpr2_120)*np.linalg.norm(B2A2_120)))
        beta2 = beta2_rd*180/pi

        #gamma3
        B3_240 = np.array([B3[0]*cos120-B3[1]*sin120, B3[0]*sin120+B3[1]*cos120, B3[2]])
        A3_240 = np.array([A3[0]*cos120-A3[1]*sin120, A3[0]*sin120+A3[1]*cos120, A3[2]])
        Cpr3_240 = np.array([0, C3[0]*sin120+C3[1]*cos120, C3[2]])
        B3A3_240 = A3_240 - B3_240
        B3Cpr3_240 = Cpr3_240 - B3_240
        beta3_rd = acos(np.dot(B3A3_240,B3Cpr3_240)/(np.linalg.norm(B3Cpr3_240)*np.linalg.norm(B3A3_240)))
        beta3 = beta3_rd*180/pi

        beta_max = 45

        is_accessible_beta = not (beta1<beta_max or beta2<beta_max or beta3<beta_max)

    if Gamma and Beta:
        return (is_accessible_beta and is_accessible_gamma)
    elif Gamma:
        return is_accessible_gamma
    elif Beta:
        return is_accessible_beta
    else:
        return True


"""Plots and test"""
def test_boundaries_calculation(z=-0.9, xmin=-1, xmax=1, ymin=-1, ymax=1, h_pointnum=50):
    """
    plots 2 figures to compare the boundaries plot with the initial cut
    :param z: z level of the horizontal cut
    :param xmin: minimum x of the horizontal cuts
    :param xmax: maximum x of the horizontal cuts
    :param ymin: minimum y of the horizontal cuts
    :param ymax: maximum y of the horizontal cuts
    :param pointnum: number of points to be considered between xmin and xmax (or ymin and ymax)
    :return: 2 plots to compare the result with the input
    """
    test = get_horiz_cut(z, xmin, xmax, ymin, ymax, h_pointnum)
    test1 = get_horiz_cut_boundaries(test)
    fig = figure()
    plot_horiz_cut(test)
    fig2 = figure()
    plot_horiz_cut(test1)
    show()

def animate_work_vol(xmin=-1, xmax=1, ymin=-1, ymax=1, zmin=-1.3, zmax=-0.02, h_pointnum=50, cutsnum = 40, pausetime = 0.2):
    """
    :param xmin: Start x position of the cubic volume to be tested
    :param xmax: End x position of the cubic volume to be tested
    :param ymin: Start y position of the cubic volume to be tested
    :param ymax: End y position of the cubic volume to be tested
    :param zmin: Start z position of the cubic volume to be tested
    :param zmax: End z position of the cubic volume to be tested
    :param h_pointnum: Number of points to be tested between xmin and max (same is used for ymin --> ymax)
    :param cutsnum: Number of horizontal cuts to be made between zmin and zmax
    :param pausetime: Time between 2 consecutive plots in seconds
    :return: animates the work volume by using cuts from top to bottom
    """
    ion()
    hold(True)
    for Z in np.linspace(zmax, zmin, cutsnum):
        cla()
        [x1, y1] = horiz_cut_to_vect(get_horiz_cut(Z, xmin, xmax, ymin, ymax, h_pointnum))
        plot(x1, y1, 'ro')
        axis([xmin, xmax, ymin, ymax])
        title("z=%1.2f" % (Z))
        pause(pausetime)
    show()

def plot_horiz_cut(horiz_cut):
    """
    :param horiz_cut: Horizontal cut to be plotted
    :return: Plots the horizontal cut in a figure
    """
    [x_vect, y_vect] = horiz_cut_to_vect(horiz_cut)
    plot(x_vect, y_vect, "ro")
    axis([horiz_cut[0, 0, 0], horiz_cut[0, 0, -1], horiz_cut[1, 0, 0], horiz_cut[1, -1, 0]])

def plot_convex_hull(xmin=-1, xmax=1, ymin=-1, ymax=1, zmin=-1.3, zmax=-0.02, h_pointnum=50, cutsnum = 40):
    """
    :param xmin: Start x position of the cubic volume to be tested
    :param xmax: End x position of the cubic volume to be tested
    :param ymin: Start y position of the cubic volume to be tested
    :param ymax: End y position of the cubic volume to be tested
    :param zmin: Start z position of the cubic volume to be tested
    :param zmax: End z position of the cubic volume to be tested
    :param h_pointnum: Number of points to be tested between xmin and max (same is used for ymin --> ymax)
    :param cutsnum: Number of horizontal cuts to be made between zmin and zmax
    :return: plots the convex hull corresponding to the work volume
    """
    [x, y, z] = get_work_volume_points(xmin, xmax, ymin, ymax, zmin, zmax, h_pointnum, cutsnum)
    allPoints = np.column_stack((x, y, z))
    hullPoints = ConvexHull(allPoints)
    fig = figure()
    ax = Axes3D(fig)
    for s in hullPoints.simplices:
        ax.plot3D(allPoints[s, 0], allPoints[s, 1], allPoints[s, 2], "r-")
    show()

def plot_mecanical_limitation(z=-0.9, Gamma=False, Beta=False):
    """
    :param z: Z axis position of the horizontal cut to be made
    :return: Plots 2 figures to compare the mathematically accessible region with the mechanically accessible one
    and see the effect of the spring.
    """
    hc = get_horiz_cut(z)
    hc1 = get_horiz_cut(z, Gamma=Gamma, Beta=Beta)
    fig = figure()
    plot_horiz_cut(hc)
    fig2 = figure()
    plot_horiz_cut(hc1)
    show()

def plot_mechanical_and_mathematical_volume(Gamma=False, Beta=False):
    fig = figure()
    ax = Axes3D(fig)
    ion()
    [x, y, z] = get_work_volume_points()
    ax.plot3D(x, y, z, ".b")
    [x1, y1, z1] = get_work_volume_points(Gamma=Gamma, Beta=Beta)
    ax.plot3D(x1, y1, z1, ".r")
    show(block=True)

"""under development"""
def iscontinuous(vect):
    """
    this function tests if there is discontinuities in a given (x,y) position
    this is used as an indicatorfor empty cavities in the work volume
    :param vect: 1 Dimensionnal vector to be tested
    :return: if there is discontinuities in the vector values
    Ex:
    [0,0,0,1,1,1,0,0] is countinuous
    [0,0,1,1,0,0,1,1,0,0] is discontinuous
    """
    result = False
    i = 0
    count = 0
    while (i < len(vect)-1):
        if not(vect[i+1] == vect[i]):
            count += 1
        i+=1
    return (count < 3)

def test_discontinuities(xmin=-1, xmax=1, ymin=-1, ymax=1, zmin=-1.3, zmax=0.02, h_pointnum=50, cutsnum=40):
    final_matrix = np.zeros((4, h_pointnum, h_pointnum, cutsnum))  # 4 dimensionnal matrix representing the space(x,y,z) + 1 to register if the point is in the work volume or not
    x_array = np.linspace(xmin, xmax, h_pointnum)  # array to fill the matrix[0] (x)
    y_array = np.linspace(ymin, ymax, h_pointnum)  # array to fill the matrix[1] (y)
    z_array = np.linspace(zmin, zmax, cutsnum)
    [x_mat, y_mat, z_mat] = np.meshgrid(x_array, y_array, z_array)
    final_matrix[0, :, :, :] = x_mat
    final_matrix[1, :, :, :] = y_mat
    final_matrix[2, :, :, :] = z_mat
    for k in range(cutsnum):
        for j in range(h_pointnum):
            for i in range(h_pointnum):
                x = final_matrix[0, i, j, k]
                y = final_matrix[1, i, j, k]
                z = final_matrix[2, i, j, k]
                res = inv_kinematics(x, y, z)
                if res == "position does not exist":
                    final_matrix[3, i, j, k] = 0
                else:
                    final_matrix[3, i, j, k] = 1
    result = True
    for i in range(h_pointnum):
        for j in range(h_pointnum):
            r = iscontinuous(final_matrix[3,i,j,:])
            if not(r):
                # print(i,j)
                result = False
    return result

def disc_num(start, end):
    for i in range(start, end):
        print(i,":",test_discontinuities(h_pointnum=i))







"""Tests"""
# animate_work_vol(pausetime=0.1, cutsnum=40)

# plot_convex_hull()

# test_boundaries_calculation(-0.4)

# disc_num(1,50)

# plot_mecanical_limitation(z=-0.5, Gamma=False, Beta=True)

# plot_mechanical_and_mathematical_volume(Beta=True)
