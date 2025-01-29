#! /usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

#Subscriber class
class laser_subscriber:
    def __init__(self):
        self.sub = rospy.Subscriber("/kobuki/laser/scan",LaserScan, self.callback )   
        self.laser_data = LaserScan() 

    def callback(self, data):
        self.laser_data = data 
    
    #Function selects only the ranges values from laser data
    def get_laser_ranges(self):
        #time.sleep(0.5)   
        return self.laser_data.ranges
        

# init publishers and subscribers
rospy.init_node("topics_quiz_node", anonymous=True)
pub1 = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
rate = rospy.Rate(2)
move = Twist()
move.linear.x = 0.0  # allows forward movement of the robot
move.angular.z = 0.0 # allows turning of the robot.

sub1 = laser_subscriber()



# function to set speed. Default set to 0
def set_speed(x_speed=0.0, z_speed=0.0):
    move.linear.x = x_speed
    move.linear.y = 0.0
    move.linear.z = 0.0
    move.angular.x = 0.0
    move.angular.y = 0.0
    move.angular.z = z_speed

def move_robot():
    min_threshold = 1.0 # minimum laser scan reading to indicate obstacle
    print("move_robot function:....")
    l_ranges = sub1.get_laser_ranges() # get laser readings. only ranges values
    len_ranges = len(l_ranges)
    x_speed = 0.3
    z_speed = 0.3
    front = int(len_ranges / 2 ) if len_ranges != 0 else 0
    right = len_ranges - 1
    left= 0.0
    if l_ranges[front] > min_threshold :
        #move straight
        if l_ranges[left] > min_threshold and l_ranges[right] > min_threshold:
            print(f'Moving straight. front reading : {l_ranges[front]}')
            set_speed(x_speed, 0.0)
        #turn left. linear.x at half normal speed
        elif l_ranges[left] < min_threshold:
            print(f'Turning left. left reading : {l_ranges[left]}')
            set_speed( x_speed , z_speed)
        #turn right . linear.x at half normal speed
        elif l_ranges[right] < min_threshold :
            print(f'Turning right. right reading : {l_ranges[right]}')
            set_speed( x_speed , -z_speed)
        else:
            pass
            print("maintaining prev action: else not straight, not turn right/left ")
    else:
    #turn left
        print(f'Turning left. left reading : {l_ranges[left]}')
        set_speed( x_speed, z_speed)
        
    # publish move data top cmd_vel topic
    pub1.publish(move) 

def main():
    while not rospy.is_shutdown(): 
        move_robot()
        rate.sleep()
    set_speed() #stop robot on exit





if __name__ == '__main__': 
    try: 
        main() 
    except rospy.ROSInterruptException: 
        pass


   
    
   








    
   


