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
from geometry_msgs.msg import PoseWithCovarianceStamped
from ieu_agv.msg import  uwb_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid

from PyQt5.QtWidgets import QApplication, QWidget, QMainWindow, QAction, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QSpacerItem, QSizePolicy, QPushButton
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import QSize

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
import matplotlib.image as mpimg

import math
import random
import numpy as np

global counter_point 
global contS
global m

global robot_realtime_pose
global occupancy_map
global lidar_ranges

global sub_map
global sub_scan
global sub_local

counter_point = 0
contS = 0

occupancy_map = None
robot_realtime_pose = None
lidar_ranges = None



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

        
        m = WidgetPlot(self)
        vlay.addWidget(m)
        self.listener()

    def listener(self):
        global sub_map
        global sub_scan
        global sub_local
        rospy.init_node('gui_icp_node', anonymous=True)
        sub_map = rospy.Subscriber("map",OccupancyGrid, self.subscribe_map)
        sub_scan = rospy.Subscriber("scan", LaserScan, self.subscribe_lidar)
        sub_local = rospy.Subscriber("localization_data_topic", Pose, self.subscribe_uwb)
        
        
    def subscribe_lidar(self,LaserScan):
        global lidar_ranges
        lidar_ranges= LaserScan


    def subscribe_uwb(self,Pose):
        global robot_realtime_pose
        robot_realtime_pose  = Pose

    def subscribe_map(self,OccupancyGrid):
        global occupancy_map
        occupancy_map= OccupancyGrid
        

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
        
        global occupancy_map
        global counter_point 
        global robot_realtime_pose
        global lidar_ranges
        ax = self.figure.add_subplot(111)

        map_width = occupancy_map.info.width

        global sub_map
        global sub_scan
        global sub_local
        
        sub_map.unregister()
        sub_scan.unregister()
        sub_local.unregister()

        processed_lidar = []
        processed_map =[]
            
        counter_map = -1 
        for i in occupancy_map.data:
            counter_map = counter_map +1 
            if i>99 : 
                x = ((counter_map%map_width)*50) -10000
                y = ((counter_map/map_width)*50) -10000
                
                processed_map.append([x,y])


        for i in range(len(processed_map)):
            ax.scatter( (processed_map[i][0]),( processed_map[i][1]), s=5 )
        
        if robot_realtime_pose != None:
            for i in range(0,360):
                if  lidar_ranges.ranges[i] <= 2:
                    ax.scatter( robot_realtime_pose.position.x+lidar_ranges.ranges[i]*math.cos(math.radians(i))*1000,robot_realtime_pose.position.y+lidar_ranges.ranges[i]*math.sin(math.radians(i))*1000, s=50 )
            ax.scatter( robot_realtime_pose.position.x ,robot_realtime_pose.position.y, s=100 )
        
        self.draw()
     

if __name__ == "__main__":
    app = QApplication(sys.argv)
    mainWin = MainWindow()
    mainWin.show()
    sys.exit( app.exec_())