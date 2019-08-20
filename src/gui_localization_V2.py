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

import sys

import rospy
import rospkg

from threading import Timer

from std_msgs.msg import String
from ieu_agv.msg import  uwb_data
from geometry_msgs.msg import Pose

from PyQt5.QtWidgets import QApplication, QWidget, QMainWindow, QAction, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QSpacerItem, QSizePolicy, QPushButton
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import QSize

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import random

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
import matplotlib.image as mpimg


global robot_realtime_pose
global contS
global m
contS = 0
global counter_point 
counter_point = 0 
class MainWindow(QMainWindow):
    def __init__(self):
        global m

        QMainWindow.__init__(self)

        self.title = 'Localization'
        self.left = 10
        self.top = 10
        self.width = 1920
        self.height = 1080

        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)

        self.statusBar().showMessage('Ready')

        mainMenu = self.menuBar()
        mainMenu.setNativeMenuBar(False)
        fileMenu = mainMenu.addMenu('File')
        helpMenu = mainMenu.addMenu('Help')

        exitButton = QAction(QIcon('exit24.png'), 'Exit', self)
        exitButton.setShortcut('Ctrl+Q')
        exitButton.setStatusTip('Exit application')
        exitButton.triggered.connect(self.close)
        fileMenu.addAction(exitButton)


        widget =  QWidget(self)
        self.setCentralWidget(widget)
        vlay = QVBoxLayout(widget)
        hlay = QHBoxLayout()
        vlay.addLayout(hlay)

        self.xLabel = QLabel('X pos:', self)
        self.xposLabel = QLabel('', self)
        #self.line = QLineEdit(self)
        self.yLabel = QLabel('Y pos:', self)
        self.yposLabel = QLabel('', self)

        hlay.addWidget(self.xLabel)
        hlay.addWidget(self.xposLabel)
        #hlay.addWidget(self.line)
        hlay.addWidget(self.yLabel)
        hlay.addWidget(self.yposLabel)
        hlay.addItem(QSpacerItem(1000, 10, QSizePolicy.Expanding))

        pybutton = QPushButton('Get Locations', self)
        pybutton.clicked.connect(self.clickMethod)
        hlay2 = QHBoxLayout()
        hlay2.addWidget(pybutton)
        hlay2.addItem(QSpacerItem(1000, 10, QSizePolicy.Expanding))
        vlay.addLayout(hlay2)
        
        m = WidgetPlot(self)
        vlay.addWidget(m)
        self.listener()

    
    def listener(self):
        rospy.init_node('gui_localization_data_node', anonymous=True)
        rospy.Subscriber("localization_data_topic", Pose, self.subscribe_data)
        
    

    def subscribe_data(self,Pose):
        global robot_realtime_pose
        robot_realtime_pose= Pose
        

    def clickMethod(self):
        global robot_realtime_pose
        self.xposLabel.setText(str(robot_realtime_pose.position.x))
        self.yposLabel.setText(str(robot_realtime_pose.position.y))

    def closeEvent(self, event):
        rospy.signal_shutdown('gui_localization_data_node')
        
class WidgetPlot(QWidget):
    def __init__(self, *args, **kwargs):
        QWidget.__init__(self, *args, **kwargs)
        self.setLayout(QVBoxLayout())
        self.canvas = PlotCanvas(self, width=10, height=8)
        self.toolbar = NavigationToolbar(self.canvas, self)
        self.layout().addWidget(self.toolbar)
        self.layout().addWidget(self.canvas)


class PlotCanvas(FigureCanvas):
    def __init__(self, parent=None, width=10, height=8, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        FigureCanvas.__init__(self, fig)
        self.setParent(parent)
        FigureCanvas.setSizePolicy(self, QSizePolicy.Expanding, QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)

        self.plot()

    def plot(self):
        ax = self.figure.add_subplot(111)
        
        #ax.plot(data, 'r-', linewidth = 0.5)
        #ax.set_title('PyQt Matplotlib Example')

        img=mpimg.imread('/home/bekir/map.pgm')
        ax.imshow(img,extent=[ -200*50, 184*50 , -200*50, 184*50 ], cmap='gist_gray')

        """
        ax.set_axisbelow(True)
        # Turn on the minor TICKS, which are required for the minor GRID
        ax.minorticks_on()
        """

        # Customize the major grid
        ax.grid(which='major', linestyle='-', linewidth='0.5', color='black')
        # Customize the minor grid
        ax.grid(which='minor', linestyle=':', linewidth='0.5', color='black')
        
        ax.set_ylim([-3000,3000])
        ax.set_xlim([-3000, 3000])
   
        self.draw()

        sensor_pos = self.read_uwb_pose()
        self.draw_anchor(sensor_pos)

        t = Timer(3, self.plotting)
        t.start()

    def draw_anchor(self,anchor_list):
        # Build a list of 4 random integers between 0 and 10 (both inclusive)
        ax = self.figure.add_subplot(111)
        
        for i in range(len(anchor_list)):
                ax.scatter(anchor_list[i][0], anchor_list[i][1],s=100,c='y')
        self.draw()
    
    def read_uwb_pose(self):
        rospack = rospkg.RosPack()
        rospack.list() 
        path_pack = rospack.get_path('ieu_agv')
        path_uwb_sens = path_pack+"/files/sensors/uwb_sensor_pos.txt"
    
        x =""
        with open(path_uwb_sens, 'r') as f:
            x = f.readlines()
        
        sensors = []
        for item in x :
            a = ((str(item).replace('\n','').split(','))) 
            a = map(int, a)
            sensors.append(a)
    
        return sensors    
    
    
 


    def plotting(self):
        global robot_realtime_pose
        global counter_point 
        ax = self.figure.add_subplot(111)


        while not rospy.is_shutdown(): 
            #self.label_Data_ContS.setText(str(contS))
            #self.label_Data_XPos.setText(str(robot_realtime_pose.position.x))
            #self.label_Data_YPos.setText(str(robot_realtime_pose.position.y))
        
            ax.scatter( robot_realtime_pose.position.x ,robot_realtime_pose.position.y, s=100 )
            self.draw()

            if counter_point == 10 :
                    ax.collections[4].remove()
            elif counter_point  < 11:
                    counter_point =counter_point +1

    

if __name__ == "__main__":
    app = QApplication(sys.argv)
    mainWin = MainWindow()
    mainWin.show()
    sys.exit( app.exec_())