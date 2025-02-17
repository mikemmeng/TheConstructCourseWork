#! /usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from find_wall.srv import FindWall, FindWallRequest 
from math import radians

####subscriber class####
class laser_subscriber:
    def __init__(self):
        self.sub = rospy.Subscriber("/scan",LaserScan, self.callback )   
        self.laser_data = LaserScan() 

    def callback(self, data):
        self.laser_data = data 
    
    #Function selects only the ranges values from laser data
    def get_laser_ranges(self):
        time.sleep(0.5)   
        return self.laser_data.ranges

#### robot_motion class ####     
class robot_motion:
    def __init__(self):
        # init publishers and subscribers
        rospy.init_node("rosject_turtlebot_node", anonymous=True)
        self.pub1 = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(1)
        self.move = Twist()
        self.sub1 = laser_subscriber()
        self.set_speed() 

    # function to set speed. Default set to 0
    def set_speed(self, x_speed=0.0, z_speed=0.0):
        self.move.linear.x = x_speed
        self.move.linear.y = 0.0
        self.move.linear.z = 0.0
        self.move.angular.x = 0.0
        self.move.angular.y = 0.0
        self.move.angular.z = z_speed

    # function to calculate the ray angle
    def calculate_ray_angle(self,index = 360):
        ray_angle = self.sub1.laser_data.angle_min + ( index * self.sub1.laser_data.angle_increment)
        print(f'calculate_ray_angle(). Ray angle value : {ray_angle}')
        return ray_angle

    #Function returns the index of first ray in ranges with  specified angle
    def get_ray(self, angle, l_ranges):
        index = 0
        ray_found_flag = False
        for ray in l_ranges:
            if(self.calculate_ray_angle(index) == radians(angle)):
                ray_found_flag= True
                break
            index+=1
        #
        rospy.loginfo(f'get_ray(). index value:{index}')
        return index if ray_found_flag else -1 # return index only if angle found. 

    #Function returns shortest ray in l_ranges 
    def get_shortest_ray(self):
        return min(self.sub1.get_laser_ranges())
    #Function returns longest ray in l_ranges
    def get_longest_ray(self):
        return max(self.sub1.get_laser_ranges())

    #Function moves robot to the wall
    #To move to wall we rotate the robot until 
    #the ray with passed index is within minimum threshold of the shortest
    def move_robot_turn_to_wall(self, index=0):
        rospy.loginfo("move_robot_turn_to_wall(): --> started")
        l_ranges = self.sub1.get_laser_ranges()
        len_ranges = len(l_ranges)
        min_threshold= 0.03
        #Change default index to front ray otherwise use passed index
        if(index==0):
            ray_index =  int( len_ranges/2 ) if len_ranges !=0 else 0 
        else:
            ray_index = int(index)
        print(f'move_robot_turn_to_wall(): ray_index: {ray_index}')
        z_speed=0.15

        #Turn robot till ray of ray_index faces wall. 
        loop_cnt, max_loop_cnt = 0, 50
        while True:
            #Get current laser readings
            l_ranges = self.sub1.get_laser_ranges()
            rospy.loginfo(f'move_robot_turn_to_wall(): --> \n l_ranges[ray_index]: {l_ranges[ray_index]} minimum l_ranges: {min(l_ranges)}')
            #Stop if ray index ray is within the min threshold to the minimum
            if l_ranges[ray_index] - min(l_ranges) <= min_threshold or loop_cnt > max_loop_cnt:
                rospy.loginfo("move_robot_turn_to_wall():--> min threshold reached :-). Stopping robot")
                #Stop robot
                self.set_speed()
                self.pub1.publish(self.move)
                break
            #Turn robot
            rospy.loginfo(f'move_robot_turn_to_wall(): --> robot turning. z_speed: {z_speed} loop count: {loop_cnt}')
            #Stop robot
            self.set_speed(0.0, z_speed)
            self.pub1.publish(self.move)
            loop_cnt+=1 
        
        rospy.loginfo("move_robot_turn_to_wall(): --> exiting")
      
    #Function moves robot forward till front ray is given distance to wall
    def move_robot_forward_to_distance(self, distance_to_wall = 0.8):
        rospy.loginfo("move_robot_forward_to_distance(): --> started")
        l_ranges = self.sub1.get_laser_ranges()
        len_ranges = len(l_ranges)
       
        front_ray =  int( len_ranges/2 ) if len_ranges !=0 else 0 
        x_speed=0.2
        #Move robot forward till distance is within distance to wall. 
        loop_cnt, max_loop_cnt = 0, 50
        while True:
            rospy.loginfo(f'move_robot_forward_to_distance(): --> front distance: {l_ranges[front_ray]} set_distance_to_wall: {distance_to_wall}')
             #get current laser scan  readings
            l_ranges = self.sub1.get_laser_ranges()
            if l_ranges[front_ray] <= distance_to_wall or loop_cnt > max_loop_cnt:
                #stop robot and exit loop
                self.set_speed()
                self.pub1.publish(self.move)
                break
            #move robot forward. 
            rospy.loginfo(f'move_robot_forward_to_distance(): --> robot moving forward. x_speed: {x_speed} loop count: {loop_cnt}')
            self.set_speed(x_speed,0.0)
            self.pub1.publish(self.move)
            loop_cnt+=1 
            
        #self.set_speed()
        #self.pub1.publish(self.move)
        rospy.loginfo("move_robot_forward_to_distance(): --> exiting")

    def move_robot(self): 
        rospy.loginfo("move_robot(): --> started") 
        turn_left_threshold = 0.2 # minimum laser scan reading to indicate obstacle
        turn_right_threshold = 0.3 #
        wall_threshold = 0.5 #
        
        l_ranges = self.sub1.get_laser_ranges()
        len_ranges = len(l_ranges)
        x_speed =0.15
        z_speed = 0.10
        front = int(len_ranges / 2 ) if len_ranges != 0 else 0
        #index= int( get_ray(90.0, l_ranges) ) 
        ray_90degrees = 80
        loop_cnt, loop_cnt_max = 0, 1000
        while True:
            l_ranges = self.sub1.get_laser_ranges() # get laser readings. only ranges values
            if l_ranges[front] > wall_threshold:
                #Move straight
                if l_ranges[ray_90degrees] >= turn_left_threshold and l_ranges[ray_90degrees] <= turn_right_threshold:
                    rospy.loginfo(f'move_robot(): moving robot straight. distance from wall reading : {l_ranges[ray_90degrees]}')
                    self.set_speed(x_speed, 0.0)
                #Turn left/away from wall. linear.x at half normal speed
                elif l_ranges[ray_90degrees] < turn_left_threshold:
                    rospy.loginfo(f'move_robot(): turning left. distance from wall reading : {l_ranges[ray_90degrees]}')
                    self.set_speed( x_speed , z_speed)
                #Turn right/towards the wall . linear.x at half normal speed
                elif l_ranges[front] > turn_right_threshold :
                    rospy.loginfo(f'move_robot(): turning right. distance from wall reading : {l_ranges[ray_90degrees]}')
                    self.set_speed( x_speed , -z_speed)
                else:
                    rospy.loginfo("move_robot(): maintaining prev action: else not straight, not turn right/left ")
            
            else:
                #Robot close to the wall. turn left rapidly
                print(f'Robot close to wall. turning left rapidly. front distance : {l_ranges[front]}')
                self.set_speed( x_speed / 2, z_speed * 10)
            #Publish move data cmd_vel topic
            self.pub1.publish(self.move)
            
            if loop_cnt > loop_cnt_max: 
                #stop robot
                self.set_speed()
                self.pub1.publish(self.move)
                break
            loop_cnt+=1
            rospy.loginfo(f'move_robot(): loop_cnt: {loop_cnt}')
            
        rospy.loginfo("move_robot(): --> exiting") 
    

#inits
robot_service_client = rospy.ServiceProxy("/robot_find_wall", FindWall ) #service client
robot_service_client_object = FindWallRequest() # service object
rosject_bot = robot_motion() #robot object. 

#service client
service_result = robot_service_client(robot_service_client_object) #Send through server connection request to FindWall service


################################################
#                                              #
# Main function                                #
#                                              #
################################################
def main():
    rospy.loginfo("main(): ---> started")
    execute_once_flag = False
    
    while not rospy.is_shutdown(): 
        #execute once commands
        if not execute_once_flag:
            if service_result:
                rosject_bot.move_robot()
            else:
                rospy.loginfo("main(): service to bring robot to start postion did not complete successfully. Program  will exit.")
                break
            execute_once_flag = True

    rospy.loginfo("main(): ---> exiting")       
    




if __name__ == '__main__': 
    try: 
        main() 
    except rospy.ROSInterruptException: 
        print("Something terrible has happened...Hope the turtlebot is okay")
        print(":-)" * 30)  
        
