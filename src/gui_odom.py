#!/usr/bin/env python
'''
__author__ = "Bekir Bostanci"
__copyright__ = ""
__credits__ = []
__license__ = ""
__version__ = "1.0.0"
__email__ = "bekirbostanci@gmail.com"
__status__ = "Production"
'''


import rospy 
from std_msgs.msg import String
from ieu_agv.msg import  uwb_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

from PyQt5 import QtCore, QtGui, QtWidgets, uic

import matplotlib
matplotlib.use('Qt5Agg')
import random

from numpy import arange, sin, pi
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

import math


#Global Variable
global data, contS, contP, str_contS, str_contP
global all_destination_id 
 
data=0
contS=0
contP=0
rate_message = 10 
global matplot 
### --- ###


class MplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=10, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        self.compute_initial_figure()

        FigureCanvas.__init__(self, fig)
        self.setParent(parent)

        FigureCanvas.setSizePolicy(self,
                                   QtWidgets.QSizePolicy.Expanding,
                                   QtWidgets.QSizePolicy.Expanding)

        FigureCanvas.updateGeometry(self)

    def compute_initial_figure(self):
        pass

class DynamicMplCanvas(MplCanvas):
    def __init__(self, *args, **kwargs):
        MplCanvas.__init__(self, *args, **kwargs)
        self.axes.set_axisbelow(True)

        # Turn on the minor TICKS, which are required for the minor GRID
        self.axes.minorticks_on()

        # Customize the major grid
        self.axes.grid(which='major', linestyle='-', linewidth='0.5', color='red')
        # Customize the minor grid
        self.axes.grid(which='minor', linestyle=':', linewidth='0.5', color='black')

   

    def update_figure(self,robot_pos):
        self.axes.scatter( robot_pos.pose.pose.position.x,  robot_pos.pose.pose.position.y, s=5)
        self.draw()


class Ui_Localization(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1000, 650)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(""), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        MainWindow.setWindowIcon(icon)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.widget = QtWidgets.QWidget(self.centralwidget)
        self.widget.setGeometry(QtCore.QRect(10, 10, 1000, 600))
        self.widget.setObjectName("widget")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.widget)
        self.verticalLayout_4.setContentsMargins(0, 0, 500, 500)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout()
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        spacerItem = QtWidgets.QSpacerItem(20, 28, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        spacerItem1 = QtWidgets.QSpacerItem(158, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_8.addItem(spacerItem1)
        self.label = QtWidgets.QLabel(self.widget)
        font = QtGui.QFont()
        font.setPointSize(18)
        font.setItalic(False)
        self.label.setFont(font)
        self.label.setObjectName("label")
        self.horizontalLayout_8.addWidget(self.label)
        spacerItem2 = QtWidgets.QSpacerItem(158, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_8.addItem(spacerItem2)
        self.verticalLayout_3.addLayout(self.horizontalLayout_8)
        self.verticalLayout_4.addLayout(self.verticalLayout_3)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
	self.label_X = QtWidgets.QLabel(self.widget)
	self.label_X.setObjectName("label_X")
	self.horizontalLayout.addWidget(self.label_X)
	self.label_Data_XPos = QtWidgets.QLabel(self.widget)
        self.label_Data_XPos.setObjectName("label_Data_XPos")
	self.horizontalLayout.addWidget(self.label_Data_XPos)
        self.horizontalLayout_5.addLayout(self.horizontalLayout)
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
	self.label_Step_Txt = QtWidgets.QLabel(self.widget)
        self.label_Step_Txt.setObjectName("label_Step_Txt")
	self.horizontalLayout_4.addWidget(self.label_Step_Txt)
	self.label_Data_ContS = QtWidgets.QLabel(self.widget)
        self.label_Data_ContS.setObjectName("label_Data_ContS")
	self.horizontalLayout_4.addWidget(self.label_Data_ContS)
        self.horizontalLayout_5.addLayout(self.horizontalLayout_4)
        self.verticalLayout.addLayout(self.horizontalLayout_5)
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
	self.label_Y = QtWidgets.QLabel(self.widget)
        self.label_Y.setObjectName("label_Y")
	self.horizontalLayout_2.addWidget(self.label_Y)
        self.label_Data_YPos = QtWidgets.QLabel(self.widget)
        self.label_Data_YPos.setObjectName("label_Data_YPos")
	self.horizontalLayout_2.addWidget(self.label_Data_YPos)
        self.horizontalLayout_6.addLayout(self.horizontalLayout_2)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
	self.label_Step = QtWidgets.QLabel(self.widget)
        self.label_Step.setObjectName("label_Step")
        self.horizontalLayout_3.addWidget(self.label_Step)
	self.label_Data_ContP = QtWidgets.QLabel(self.widget)
        self.label_Data_ContP.setObjectName("label_Data_ContP")
        self.horizontalLayout_3.addWidget(self.label_Data_ContP)
        self.horizontalLayout_6.addLayout(self.horizontalLayout_3)
        self.verticalLayout.addLayout(self.horizontalLayout_6)
        self.verticalLayout_2.addLayout(self.verticalLayout)
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.verticalLayout_2.addLayout(self.horizontalLayout_7)
        self.verticalLayout_4.addLayout(self.verticalLayout_2)
        MainWindow.setCentralWidget(self.centralwidget)

        #QT Matplot
        self.matplot = DynamicMplCanvas(self.widget, width=10, height=5, dpi=100)
        self.matplot.move(0,100)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)


    def fileQuit(self):
        self.close()

    def closeEvent(self, ce):
        self.fileQuit()
                

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Odometry Position"))
        self.label.setText(_translate("MainWindow", "IEU_AGV Odometry Position"))
        self.label_X.setText(_translate("MainWindow", "X Position:"))
        self.label_Data_XPos.setText(_translate("MainWindow", "0"))
        self.label_Step_Txt.setText(_translate("MainWindow", "Step:"))
        self.label_Data_ContS.setText(_translate("MainWindow", "0"))
        self.label_Y.setText(_translate("MainWindow", "Y Position:"))
        self.label_Data_YPos.setText(_translate("MainWindow", "0"))
        self.label_Step.setText(_translate("MainWindow", ""))
        self.label_Data_ContP.setText(_translate("MainWindow", ""))
	
	self.listener()

    def listener(self):
        rospy.init_node('gui_pose_node', anonymous=True)
        rospy.Subscriber("odom", Odometry, self.subscribe_data)

    def subscribe_data(self,Odometry):
        global contS, str_contS
        contS=contS+1

        if contS%rate_message==0:
                contS =0
                str_contS=str(contS)
                self.label_Data_ContS.setText(str_contS)
                robot_pos= Odometry
                self.label_Data_XPos.setText(str(robot_pos.pose.pose.position.x))
                self.label_Data_YPos.setText(str(robot_pos.pose.pose.position.y))
                self.matplot.update_figure(robot_pos)

   
	
if __name__ == "__main__":
    import sys

    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_Localization()
    ui.setupUi(MainWindow)
    MainWindow.show()
    
    sys.exit(app.exec_())
    rospy.spin()