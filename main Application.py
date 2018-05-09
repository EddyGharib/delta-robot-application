"""
This software was developed under python 3.4.1
It depends on the following packages:
    - numpy
    - matplotlib
    - scipy
    - PyQt4
please make sure these packages are installed before you lunch this app
"""

from PyQt4 import QtGui,QtCore

from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QT as NavigationToolbar

from delta_work_volume import *
from delta_plot import *


class Window(QtGui.QMainWindow):
    def __init__(self, parent=None):
        super(Window, self).__init__(parent)

        # Generate the points of the work volume
        start_time = time.clock()
        self.generateWorkVolumeData()
        print("--- %s seconds ---" % (time.clock() - start_time))

        # Start the gui (Main window creation)
        start_time = time.clock()
        self.setupUI()
        print("--- %s seconds ---" % (time.clock() - start_time))



    def setupUI(self):
        self.setGeometry(40, 40, 1500, 800)
        self.showMaximized()
        self.setWindowTitle("Delta Robot application")

        # Update delta robot params
        self.drob_dimensions = default_drob_dimensions

        # work volume figure
        self.work_vol_figure = Figure()
        self.work_vol_canvas = FigureCanvas(self.work_vol_figure)
        self.work_vol_toolbar = NavigationToolbar(self.work_vol_canvas, self)
        self.work_vol_ax = Axes3D(self.work_vol_figure)
        self.work_vol_canvas.callbacks.connect('button_release_event',self.updateCoords)

        # Delta robot figure
        self.delta_rob_figure = Figure()
        self.delta_rob_canvas = FigureCanvas(self.delta_rob_figure)
        self.delta_rob_toolbar = NavigationToolbar(self.delta_rob_canvas,self)
        self.delta_rob_ax = Axes3D(self.delta_rob_figure)
        draw_delta_robot(30, 30, 30, self.delta_rob_ax)
        self.delta_rob_canvas.draw()

        # Generating gamma and beta checkboxes layout
        self.bound_checkbox = QtGui.QCheckBox('work volume')
        self.bound_checkbox.setCheckState(True)
        self.bound_checkbox.setTristate(False)
        self.bound_checkbox.stateChanged.connect(self.updateWorkVolume)
        self.bound_points_generated = True

        self.gamma_checkbox = QtGui.QCheckBox('Gamma limitation')
        self.gamma_checkbox.stateChanged.connect(self.updateWorkVolume)
        self.gamma_points_generated = False

        self.beta_checkbox = QtGui.QCheckBox('Beta limitation')
        self.beta_checkbox.stateChanged.connect(self.updateWorkVolume)
        self.beta_points_generated = False

        self.gamma_beta_checkbox = QtGui.QCheckBox('Gamma and Beta limitation')
        self.gamma_beta_checkbox.stateChanged.connect(self.updateWorkVolume)
        self.gamma_beta_points_generated = False

        gammabetaLayout = QtGui.QHBoxLayout()
        gammabetaLayout.addWidget(self.bound_checkbox)
        gammabetaLayout.addWidget(self.gamma_checkbox)
        gammabetaLayout.addWidget(self.beta_checkbox)
        gammabetaLayout.addWidget(self.gamma_beta_checkbox)
        self.updateWorkVolume()

        # Generating the left layout
        leftLayout = QtGui.QVBoxLayout()
        leftLayout.addLayout(gammabetaLayout)
        leftLayout.addWidget(self.work_vol_toolbar)
        leftLayout.addWidget(self.work_vol_canvas)

        # Generating the xyzLayout
        self.x_label = QtGui.QLabel('x:')
        self.y_label = QtGui.QLabel('y:')
        self.z_label = QtGui.QLabel('z:')
        self.x_label.setAlignment(QtCore.Qt.AlignRight)
        self.y_label.setAlignment(QtCore.Qt.AlignRight)
        self.z_label.setAlignment(QtCore.Qt.AlignRight)

        self.x_edit = QtGui.QTextEdit()
        self.y_edit = QtGui.QTextEdit()
        self.z_edit = QtGui.QTextEdit()
        self.x_edit.setFixedSize(100,30)
        self.y_edit.setFixedSize(100,30)
        self.z_edit.setFixedSize(100,30)
        self.x_edit.setReadOnly(True)
        self.y_edit.setReadOnly(True)
        self.z_edit.setReadOnly(True)

        xyzLayout = QtGui.QHBoxLayout()
        xyzLayout.addWidget(self.x_label)
        xyzLayout.addWidget(self.x_edit)
        xyzLayout.addWidget(self.y_label)
        xyzLayout.addWidget(self.y_edit)
        xyzLayout.addWidget(self.z_label)
        xyzLayout.addWidget(self.z_edit)

        # Generating the t123Layout
        self.t1_label = QtGui.QLabel('t1:')
        self.t2_label = QtGui.QLabel('t2:')
        self.t3_label = QtGui.QLabel('t3:')
        self.t1_label.setAlignment(QtCore.Qt.AlignRight)
        self.t2_label.setAlignment(QtCore.Qt.AlignRight)
        self.t3_label.setAlignment(QtCore.Qt.AlignRight)

        self.t1_edit = QtGui.QTextEdit()
        self.t2_edit = QtGui.QTextEdit()
        self.t3_edit = QtGui.QTextEdit()
        self.t1_edit.setFixedSize(100,30)
        self.t2_edit.setFixedSize(100,30)
        self.t3_edit.setFixedSize(100,30)
        self.t1_edit.setReadOnly(True)
        self.t2_edit.setReadOnly(True)
        self.t3_edit.setReadOnly(True)

        t123Layout = QtGui.QHBoxLayout()
        t123Layout.addWidget(self.t1_label)
        t123Layout.addWidget(self.t1_edit)
        t123Layout.addWidget(self.t2_label)
        t123Layout.addWidget(self.t2_edit)
        t123Layout.addWidget(self.t3_label)
        t123Layout.addWidget(self.t3_edit)

        rightLayout = QtGui.QVBoxLayout()
        rightLayout.addLayout(xyzLayout)
        rightLayout.addLayout(t123Layout)
        rightLayout.addWidget(self.delta_rob_toolbar)
        rightLayout.addWidget(self.delta_rob_canvas)

        # Generating main layout
        mainLayout = QtGui.QHBoxLayout()
        mainLayout.addLayout(leftLayout)
        mainLayout.addLayout(rightLayout)
        wid = QtGui.QWidget(self)
        self.setCentralWidget(wid)
        wid.setLayout(mainLayout)

        # Menu Bar configuration

        # delta Robot Params
        bar = self.menuBar()
        delta_rob_menuBar = bar.addMenu("delta Params")
        set_delta_robot_action = QtGui.QAction("Set parameters", self)
        set_delta_robot_action.triggered.connect(self.set_delta_rob_params)
        delta_rob_menuBar.addAction(set_delta_robot_action)

    def updateWorkVolume(self):
        # Robot dimensions
        d = self.drob_dimensions['d']
        R = self.drob_dimensions['R']
        r = self.drob_dimensions['r']
        l = self.drob_dimensions['l']
        ion()
        self.work_vol_ax.cla()
        self.delta_rob_ax.cla()
        if (fwd_kinematics(30,30,30,self.drob_dimensions) != "position does not exist"):
            draw_delta_robot(30, 30, 30, self.delta_rob_ax, self.drob_dimensions)
        self.delta_rob_canvas.draw()
        z_min = - r - l
        z_max = get_z_max(self.drob_dimensions)
        x_max = l
        y_min = - self.drob_dimensions['R'] - self.drob_dimensions['r'] - self.drob_dimensions['l']
        if self.bound_checkbox.isChecked():
            if not self.bound_points_generated:
                [self.x_bound, self.y_bound, self.z_bound] = get_work_volume_points(xmin=-x_max, xmax=x_max, ymin=y_min, ymax=-y_min, zmin = z_min, zmax=z_max, drob_dimensions=self.drob_dimensions)
                self.bound_points_generated = True
            self.work_vol_ax.plot3D(self.x_bound, self.y_bound, self.z_bound, ".b")
        if self.gamma_checkbox.isChecked():
            if not self.gamma_points_generated:
                [self.x_gamma_bound, self.y_gamma_bound, self.z_gamma_bound] = get_work_volume_points(boundary = True, Gamma = True, Beta = False, xmin=-x_max, xmax=x_max, ymin=y_min, ymax=-y_min, zmin = z_min, zmax=z_max, drob_dimensions=self.drob_dimensions)
                self.gamma_points_generated = True
            self.work_vol_ax.plot3D(self.x_gamma_bound, self.y_gamma_bound, self.z_gamma_bound, ".r")
        if self.beta_checkbox.isChecked():
            if not self.beta_points_generated:
                [self.x_beta_bound, self.y_beta_bound, self.z_beta_bound] = get_work_volume_points(boundary = True, Gamma = False, Beta = True, xmin=-x_max, xmax=x_max, ymin=y_min, ymax=-y_min, zmin = z_min, zmax=z_max, drob_dimensions=self.drob_dimensions)
                self.beta_points_generated = True
            self.work_vol_ax.plot3D(self.x_beta_bound, self.y_beta_bound, self.z_beta_bound, ".k")
        if self.gamma_beta_checkbox.isChecked():
            if not self.gamma_beta_points_generated:
                [self.x_gamma_beta_bound, self.y_gamma_beta_bound, self.z_gamma_beta_bound] = get_work_volume_points(boundary = True, Gamma = True, Beta = True, xmin=-x_max, xmax=x_max, ymin=y_min, ymax=-y_min, zmin = z_min, zmax=z_max, drob_dimensions=self.drob_dimensions)
                self.gamma_beta_points_generated = True
            self.work_vol_ax.plot3D(self.x_gamma_beta_bound, self.y_gamma_beta_bound, self.z_gamma_beta_bound, ".g")
        self.work_vol_canvas.draw()

    def updateCoords(self,event):
        """
        This function is called when the mouse is released inside the work volume plot area
        :param event: parameter holding all the details about the release event
        :return: updates the values of (x,y,z) labels, (t1,t2,t3) labels and the plot of the robot
        """
        x = event.xdata
        y = event.ydata
        s = self.work_vol_ax.format_coord(x,y)
        [a, b, c] = s.split(',')
        a = a[(a.find('=')+1):]
        b = b[(b.find('=')+1):]
        c = c[(c.find('=')+1):]
        self.x_edit.setText(a)
        self.y_edit.setText(b)
        self.z_edit.setText(c)
        a = float(a)
        b = float(b)
        c = float(c)
        res = inv_kinematics(a, b, c)
        if(res == "position does not exist"):
            t1 = t2 = t3 = "Non existing pt"
        else:
            t1 = res[0]
            t2 = res[1]
            t3 = res[2]
            draw_delta_robot(t1, t2, t3, self.delta_rob_ax)
            self.delta_rob_canvas.draw()
        self.t1_edit.setText(str(t1))
        self.t2_edit.setText(str(t2))
        self.t3_edit.setText(str(t3))

        return

    def generateWorkVolumeData(self):
        [self.x_bound, self.y_bound, self.z_bound] = get_work_volume_points()

    def set_delta_rob_params(self):
        dialog = UI_delta_Robot_Params(self.drob_dimensions)
        if dialog.exec_():
            self.drob_dimensions['R'] = float(dialog.returned_vals['R'])
            self.drob_dimensions['r'] = float(dialog.returned_vals['r'])
            self.drob_dimensions['l'] = float(dialog.returned_vals['l'])
            self.drob_dimensions['d'] = float(dialog.returned_vals['d'])
            self.bound_points_generated = False
            self.gamma_points_generated = False
            self.beta_points_generated = False
            self.gamma_beta_points_generated = False
            self.updateWorkVolume()

class UI_delta_Robot_Params(QtGui.QDialog):
    def __init__(self, drob_dimensions = default_drob_dimensions):
        # Robot dimensions
        self.d = drob_dimensions['d']
        self.R = drob_dimensions['R']
        self.r = drob_dimensions['r']
        self.l = drob_dimensions['l']

        QtGui.QDialog.__init__(self)
        self.setupUI()
        self.returned_vals = {}


    def setupUI(self):
        # Create R Layout
        R_layout = QtGui.QHBoxLayout()
        self.R_label = QtGui.QLabel('R:')
        self.R_label.setAlignment(QtCore.Qt.AlignRight)
        self.R_edit = QtGui.QTextEdit()
        self.R_edit.setFixedSize(100,30)
        self.R_edit.setText(str(self.R))
        R_layout.addWidget(self.R_label)
        R_layout.addWidget(self.R_edit)

        # Create r Layout
        r_layout = QtGui.QHBoxLayout()
        self.r_label = QtGui.QLabel('r:')
        self.r_label.setAlignment(QtCore.Qt.AlignRight)
        self.r_edit = QtGui.QTextEdit()
        self.r_edit.setFixedSize(100,30)
        self.r_edit.setText(str(self.r))
        r_layout.addWidget(self.r_label)
        r_layout.addWidget(self.r_edit)

        # Create l Layout
        l_layout = QtGui.QHBoxLayout()
        self.l_label = QtGui.QLabel('l:')
        self.l_label.setAlignment(QtCore.Qt.AlignRight)
        self.l_edit = QtGui.QTextEdit()
        self.l_edit.setFixedSize(100,30)
        self.l_edit.setText(str(self.l))
        l_layout.addWidget(self.l_label)
        l_layout.addWidget(self.l_edit)

        # Create l Layout
        d_layout = QtGui.QHBoxLayout()
        self.d_label = QtGui.QLabel('d:')
        self.d_label.setAlignment(QtCore.Qt.AlignRight)
        self.d_edit = QtGui.QTextEdit()
        self.d_edit.setFixedSize(100,30)
        self.d_edit.setText(str(self.d))
        d_layout.addWidget(self.d_label)
        d_layout.addWidget(self.d_edit)

        # Create the buttons
        self.ok_button = QtGui.QPushButton("Ok")
        self.ok_button.clicked.connect(self.submitclose)
        self.Cancel = QtGui.QPushButton("Cancel")
        self.Cancel.clicked.connect(self.cancel_clicked)
        self.Default = QtGui.QPushButton("Default")
        self.Default.clicked.connect(self.default_clicked)
        buttons_layout = QtGui.QHBoxLayout()
        buttons_layout.addWidget(self.ok_button)
        buttons_layout.addWidget(self.Default)
        buttons_layout.addWidget(self.Cancel)

        overall_layout = QtGui.QVBoxLayout()
        overall_layout.addLayout(R_layout)
        overall_layout.addLayout(r_layout)
        overall_layout.addLayout(l_layout)
        overall_layout.addLayout(d_layout)
        overall_layout.addLayout(buttons_layout)

        self.setLayout(overall_layout)

    def submitclose(self):
        self.returned_vals['r'] = self.r_edit.toPlainText()
        self.returned_vals['R'] = self.R_edit.toPlainText()
        self.returned_vals['l'] = self.l_edit.toPlainText()
        self.returned_vals['d'] = self.d_edit.toPlainText()
        self.accept()

    def cancel_clicked(self):
        self.accept()

    def default_clicked(self):
        self.r_edit.setText(str(c_default_drob_dimensions['r']))
        self.R_edit.setText(str(c_default_drob_dimensions['R']))
        self.l_edit.setText(str(c_default_drob_dimensions['l']))
        self.d_edit.setText(str(c_default_drob_dimensions['d']))
        # print(str(default_drob_dimensions['r']))


if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    main = Window()
    main.show()
    sys.exit(app.exec_())
