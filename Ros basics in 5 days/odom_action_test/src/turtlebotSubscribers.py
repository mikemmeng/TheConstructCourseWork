#! /usr/bin/env python

import rospy
import time
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


#laser scan subscriber
class LaserSubscriber:
    def __init__(self):
        self.laser_subscriber = rospy.Subscriber("/scan",LaserScan, self.callback )   
        self.laser_data = LaserScan() 
       
    def callback(self, data):
        self.laser_data = data 
    
    #Function selects only the ranges values from laser data
    def get_laser_ranges(self):
        time.sleep(0.5)  
        return self.laser_data.ranges
        
#odom reading subscriber
class OdometrySubscriber:
    def __init__(self):
        self.odometry_subscriber = rospy.Subscriber("/odom", Odometry, self.callback )   
        self.odometry_data = Odometry()

    def callback(self, data):
        self.odometry_data = data

    def get_odometry_data(self):
        time.sleep(0.5)
        return [self.odometry_data.pose.pose.position.x , self.odometry_data.pose.pose.position.y, self.odometry_data.pose.pose.position.z]
      

