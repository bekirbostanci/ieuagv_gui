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


import rospy, time
from std_msgs.msg import String
from ieu_agv.msg import  uwb_data
from geometry_msgs.msg import PoseWithCovarianceStamped 
from geometry_msgs.msg import PoseStamped 
from geometry_msgs.msg import Twist 

from PyQt5 import QtCore, QtGui, QtWidgets, uic

import random

from numpy import arange, sin, pi

import math


## Global Variable
### --- ###
class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.setEnabled(True)
        MainWindow.resize(845, 280)
        self.centralWidget = QtWidgets.QWidget(MainWindow)
        self.centralWidget.setEnabled(True)
        self.centralWidget.setObjectName("centralWidget")
        self.verticalLayoutWidget = QtWidgets.QWidget(self.centralWidget)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(10, 20, 831, 221))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(11, 11, 11, 11)
        self.verticalLayout.setSpacing(6)
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setContentsMargins(10, 11, 10, 11)
        self.horizontalLayout.setSpacing(6)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label.setObjectName("label")
        self.horizontalLayout.addWidget(self.label)
        self.txtFinalX = QtWidgets.QLineEdit(self.verticalLayoutWidget)
        self.txtFinalX.setObjectName("txtFinalX")
        self.horizontalLayout.addWidget(self.txtFinalX)
        self.label_2 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_2.setObjectName("label_2")
        self.horizontalLayout.addWidget(self.label_2)
        self.txtFinalY = QtWidgets.QLineEdit(self.verticalLayoutWidget)
        self.txtFinalY.setObjectName("txtFinalY")
        self.horizontalLayout.addWidget(self.txtFinalY)
        self.label_7 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_7.setObjectName("label_7")
        self.horizontalLayout.addWidget(self.label_7)
        self.txtFinalAngle = QtWidgets.QLineEdit(self.verticalLayoutWidget)
        self.txtFinalAngle.setObjectName("txtFinalAngle")
        self.horizontalLayout.addWidget(self.txtFinalAngle)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setContentsMargins(10, 11, 10, 11)
        self.horizontalLayout_4.setSpacing(6)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.label_4 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_4.setObjectName("label_4")
        self.horizontalLayout_4.addWidget(self.label_4)
        self.lblPosX = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.lblPosX.setText("")
        self.lblPosX.setObjectName("lblPosX")
        self.horizontalLayout_4.addWidget(self.lblPosX)
        self.label_3 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_3.setObjectName("label_3")
        self.horizontalLayout_4.addWidget(self.label_3)
        self.lblPosY = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.lblPosY.setText("")
        self.lblPosY.setObjectName("lblPosY")
        self.horizontalLayout_4.addWidget(self.lblPosY)
        self.label_9 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_9.setObjectName("label_9")
        self.horizontalLayout_4.addWidget(self.label_9)
        self.lblAngle = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.lblAngle.setText("")
        self.lblAngle.setObjectName("lblAngle")
        self.horizontalLayout_4.addWidget(self.lblAngle)
        self.verticalLayout.addLayout(self.horizontalLayout_4)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setContentsMargins(11, 11, 11, 11)
        self.horizontalLayout_2.setSpacing(6)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.label_13 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_13.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_13.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_13.setObjectName("label_13")
        self.horizontalLayout_2.addWidget(self.label_13)
        self.lblVelX = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.lblVelX.setText("")
        self.lblVelX.setObjectName("lblVelX")
        self.horizontalLayout_2.addWidget(self.lblVelX)
        self.label_15 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_15.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_15.setObjectName("label_15")
        self.horizontalLayout_2.addWidget(self.label_15)
        self.lblVelY = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.lblVelY.setText("")
        self.lblVelY.setObjectName("lblVelY")
        self.horizontalLayout_2.addWidget(self.lblVelY)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_5.setContentsMargins(11, 11, 11, 11)
        self.horizontalLayout_5.setSpacing(6)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.btnGoPos = QtWidgets.QPushButton(self.verticalLayoutWidget)
        self.btnGoPos.setObjectName("btnGoPos")
        self.horizontalLayout_5.addWidget(self.btnGoPos)
        self.verticalLayout.addLayout(self.horizontalLayout_5)
        MainWindow.setCentralWidget(self.centralWidget)
        self.mainToolBar = QtWidgets.QToolBar(MainWindow)
        self.mainToolBar.setObjectName("mainToolBar")
        MainWindow.addToolBar(QtCore.Qt.TopToolBarArea, self.mainToolBar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Go Position "))
        self.label.setText(_translate("MainWindow", "Final X :"))
        self.label_2.setText(_translate("MainWindow", "Final Y :"))
        self.label_7.setText(_translate("MainWindow", "Final Angle : "))
        self.label_4.setText(_translate("MainWindow", "Final Position X : "))
        self.label_3.setText(_translate("MainWindow", "Final Position Y :"))
        self.label_9.setText(_translate("MainWindow", "Final Angle : "))
        self.label_13.setText(_translate("MainWindow", "Vel X : "))
        self.label_15.setText(_translate("MainWindow", "Vel Y: "))
        self.btnGoPos.setText(_translate("MainWindow", "Go Position"))


        self.btnGoPos.clicked.connect(self.setGoal)
        rospy.init_node('gui_localization_data_node', anonymous=True)
        self.listener()

    def setGoal(self):
        goal_publisher = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=5)

        goal = PoseStamped()

        goal.header.seq = 1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"

        goal.pose.position.x = float(self.txtFinalX.text())
        goal.pose.position.y = float(self.txtFinalY.text())
        goal.pose.position.z = 0.0

        theta = float(self.txtFinalAngle.text())
        theta = theta*math.pi
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = math.sin(theta/2)
        goal.pose.orientation.w = math.cos(theta/2)

        rospy.sleep(1)
        goal_publisher.publish(goal)


    def listener(self):
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.subscribe_data_pose)
        rospy.Subscriber("cmd_vel", Twist, self.subscribe_data_velocity)
    
    def subscribe_data_pose(self,PoseWithCovarianceStamped):
        self.lblPosX.setText(str(PoseWithCovarianceStamped.pose.pose.position.x))
        self.lblPosY.setText(str(PoseWithCovarianceStamped.pose.pose.position.y))
        self.lblAngle.setText(str(PoseWithCovarianceStamped.pose.pose.orientation.w))
    
    def subscribe_data_velocity(self,Twist):
        self.lblVelX.setText(str(Twist.linear.x))
        self.lblVelY.setText(str(Twist.linear.y))
    

    def fileQuit(self):
        self.close()

    def closeEvent(self, ce):
        self.fileQuit()
	
if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    
    sys.exit(app.exec_())
    rospy.spin()