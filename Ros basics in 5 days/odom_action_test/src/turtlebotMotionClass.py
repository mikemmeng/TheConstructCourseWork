#! /usr/bin/env python

import rospy
from math import radians, sqrt
from geometry_msgs.msg import Twist
from find_wall.srv import FindWall, FindWallRequest 
from turtlebotSubscribers import LaserSubscriber, OdometrySubscriber



    
class turtlebotMotion():

    def __init__(self):

        #init publishers
        self.turtlebot_cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(1)
        self.turtlebot_move = Twist()
        
        #init subscribers
        self.laser_subscriber = LaserSubscriber()
        self.odometry_subscriber = OdometrySubscriber()
        
        #class attributes
        self.turn_left_threshold = 0.3 # minimum laser scan reading to indicate obstacle
        self.turn_right_threshold = 0.5 #
        self.wall_threshold = 0.5 #minimum distance to wall 
        self.odom_distance_delta_accept_minimum = 0.001 #minimum acceptable delta value. 
        self.x_speed = 0.15 #forward speed in m/s
        self.z_speed = 0.1 #turn speed in m/s
        
        self.list_of_odoms = [] 
        self.list_of_distance_deltas = []
        self.list_max_elements  = 1000
        #self.list_current_total_odoms = [0.0,0.0,0.0] #x, y, z 
        self.odom_total_distance_x= 0

        #function inits
        self._set_speed() #stop robot 

    #Function to set speed. default set to 0.
    def _set_speed(self, x_speed=0.0, z_speed=0.0):
        self.turtlebot_move.linear.x = x_speed
        self.turtlebot_move.linear.y = 0.0
        self.turtlebot_move.linear.z = 0.0
        self.turtlebot_move.angular.x = 0.0
        self.turtlebot_move.angular.y = 0.0
        self.turtlebot_move.angular.z = z_speed
    
    #Function to publish to cmd_vel topic
    def _set_publisher_cmd_vel(self):
        #check publisher connection and publish msg
        #rospy.loginfo("[_set_publisher_cmd_vel()]: started. publishing cmd_vel topic msg")
        while self.turtlebot_cmd_vel_publisher.get_num_connections() < 1:
            pass
        self.turtlebot_cmd_vel_publisher.publish(self.turtlebot_move)
        #rospy.loginfo("[_set_publisher_cmd_vel()]: exiting. publishing cmd_vel topic msg successful")

    #Function to calculate the ray angle
    def calculate_ray_angle(self,index = 360):
        ray_angle = self.laser_subscriber.laser_data.angle_min + ( index * self.laser_subscriber.laser_data.angle_increment)
        print(f"calculate_ray_angle(). Ray angle value : {ray_angle}")
        return ray_angle

    #Function returns the index of first ray in ranges with specified angle
    def _get_ray(self, angle, l_ranges):
        index = 0
        ray_found_flag = False
        for ray in l_ranges:
            if(self.calculate_ray_angle(index) == radians(angle)):
                ray_found_flag= True
                break
            index+=1
        #
        rospy.loginfo(f"get_ray(). index value:{index}")
        return index if ray_found_flag else -1 # return index only if angle found. 

    #Function returns shortest ray in l_ranges 
    def _get_shortest_ray(self):
        return min(self.laser_subscriber.get_laser_ranges())

    #Function returns longest ray in l_ranges
    def _get_longest_ray(self):
        return max(self.laser_subscriber.get_laser_ranges())

    #Function updates the odometry list
    def _set_odometry(self):
        rospy.loginfo("[_set_odometry()]: started")
        self.list_of_odoms.append(self.odometry_subscriber.get_odometry_data())
        #trim list to maintain only last 100 entries
        if len(self.list_of_odoms) > self.list_max_elements:
            self.list_of_odoms.pop(0)
        rospy.loginfo(f"[_set_odometry()]: list_of_odoms:{self.list_of_odoms} ")
        rospy.loginfo("[_set_odometry()]: exiting")
    
    #Function calculates the distance deltas between two odom readings
    def _calculate_odometry(self, x0,y0,x1,y1):
        #calculate delta
        distance_delta = sqrt( (x1-x0)*(x1-x0) + (y1-y0)*(y1-y0) )
        #update list of deltas. Only update if difference value is above move threshold
        # if robot is stuck or not moving we should discard this delta values
        self.list_of_distance_deltas.append(distance_delta)
        if len(self.list_of_distance_deltas) > self.list_max_elements:
            self.list_of_distance_deltas.pop(0)
        return distance_delta

    #Function updates the distance covered by adding up distance deltas.
    def _set_odometry_total(self, x0,y0,x1,y1):
        #rospy.loginfo("[_set_odometry_total()]: started")
        #Only update if difference value is above move threshold.
        # If robot is stuck or not moving we should discard this delta values
        distance_delta = self._calculate_odometry(x0,y0,x1,y1)
        if distance_delta > self.odom_distance_delta_accept_minimum :
            self.odom_total_distance_x += distance_delta
        
        #rospy.loginfo("[_set_odometry_total()]: exiting")



    #Function moves robot to the wall
    #To move to wall we rotate the robot until 
    #the ray with passed index is within minimum threshold of the shortest
    def move_turn_to_wall(self, index=0):
        rospy.loginfo("[move_turn_to_wall()]: started")
        l_ranges = self.laser_subscriber.get_laser_ranges()
        len_ranges = len(l_ranges)
        min_threshold= 0.03
        #Change default index to front ray otherwise use passed index
        if(index==0):
            ray_index =  int( len_ranges/2 ) if len_ranges !=0 else 0 
        else:
            ray_index = int(index)
        rospy.loginfo(f"[move_turn_to_wall()]: ray_index: {ray_index}")
        z_speed=0.2

        #Turn robot till ray of ray_index faces wall. 
        loop_cnt, max_loop_cnt = 0, 60
        while True:
            #Get current laser readings
            l_ranges = self.laser_subscriber.get_laser_ranges()
            rospy.loginfo(f"[move_turn_to_wall()]:  \n l_ranges[ray_index]: {l_ranges[ray_index]} minimum l_ranges: {min(l_ranges)}")
            #Stop if ray index ray is within the min threshold to the minimum
            if l_ranges[ray_index] - min(l_ranges) <= min_threshold or loop_cnt > max_loop_cnt:
                rospy.loginfo("[move_turn_to_wall()]: min threshold reached :-). Stopping robot")
                #Stop robot
                self._set_speed()
                self._set_publisher_cmd_vel()
                break
            #Turn robot
            rospy.loginfo(f"[move_turn_to_wall()]: robot turning. z_speed: {z_speed} loop count: {loop_cnt}")
            #Stop robot
            self._set_speed(0.0, z_speed)
            self._set_publisher_cmd_vel()
            loop_cnt+=1 
        
        rospy.loginfo("[move_turn_to_wall()]: exiting")
    
    #Function to move robot forward by defined speed
    def move_forward(self, x_speed=0.0,z_speed=0.0 ):
        #Set speeds
        self._set_speed(x_speed,z_speed)
        #Publish to cmd_vel topic
        self._set_publisher_cmd_vel()  


    #Function moves robot forward till front ray is given distance to wall
    def move_forward_to_distance(self, distance_to_wall = 0.5):
        rospy.loginfo("[move_forward_to_distance()]: started")
        l_ranges = self.laser_subscriber.get_laser_ranges()
        len_ranges = len(l_ranges)
       
        front_ray =  int( len_ranges/2 ) if len_ranges !=0 else 0 
        #Move robot forward till distance is within distance to wall. 
        loop_cnt, max_loop_cnt = 0, 50
        while True:
            rospy.loginfo(f"[move_forward_to_distance()]: front distance: {l_ranges[front_ray]} set_distance_to_wall: {distance_to_wall}")
             #get current laser scan  readings
            l_ranges = self.laser_subscriber.get_laser_ranges()
            if l_ranges[front_ray] <= distance_to_wall or loop_cnt > max_loop_cnt:
                #stop robot and exit loop
                self.move_forward()
                break
            #move robot forward. 
            rospy.loginfo(f"[move_forward_to_distance():] --> robot moving forward. x_speed: {self.x_speed} loop count: {loop_cnt}")
            self.move_forward(self.x_speed)
            loop_cnt+=1 
            
        rospy.loginfo("[move_forward_to_distance()]: exiting")
    #Function moves robot forward by a given distance
    def move_forward_by_distance(self, start_odom_values = [], distance_to_travel=10.0 ):
        rospy.loginfo("[move_forward_by_distance()]: --> started") 
       
        #initialize start odometry values
        if len(start_odom_values) > 0:
            odom_x0 = start_odom_values[0]
            odom_y0 = start_odom_values[1]
        else:
            odom_x0 = 0
            odom_y0 = 0
        odom_x1 = odom_x0
        odom_y1 = odom_y0
        odom_current = []
        current_distance = 0
   
        if current_distance < distance_to_travel:
            #move forward
            self.move_forward(self.x_speed)
            self.rate.sleep()
            #get current odometry
            odom_current = self.odometry_subscriber.get_odometry_data()
            odom_x1 = odom_current[0]
            odom_y1 = odom_current[1]

            #calculate distance
            current_distance = self._set_odometry_total(odom_x0,odom_y0,odom_x1,odom_y1)
            #update 
            self.odom_total_distance_x = current_distance
        else:
            #stop robot
            self.move_forward()
        rospy.loginfo("[move_forward_by_distance()]: --> exiting") 
        
    def move_forward_by_distance_follow_wall(self, start_odom_values = [], distance_to_travel=10.0 ):
        #rospy.loginfo("[move_forward_by_distance_follow_wall()]: --> started") 
       
        #initialize start odometry values
        if len(start_odom_values) > 0:
            odom_x0 = start_odom_values[0]
            odom_y0 = start_odom_values[1]
        else:
            odom_x0 = 0
            odom_y0 = 0
        odom_x1 = odom_x0
        odom_y1 = odom_y0
        current_distance = 0
   
        if current_distance < distance_to_travel:
            #Move forward while following the wall
            #self.move_forward(self.x_speed)
            self.move_follow_wall()
            self.rate.sleep()

            #get current odometry. Update odometry list of odoms
            self._set_odometry()
            #use last list in updated list of odoms for x1 and y1
            list_length= len(self.list_of_odoms)
            if list_length > 0:
                odom_x1 = self.list_of_odoms[list_length-1][0] #first element in last list of list of odoms
                odom_y1 = self.list_of_odoms[list_length-1][1] #second element in last list of list of odoms
            else:
                #keep start odom values if list of odoms is empty
                odom_x1 = odom_x0
                odom_y1 = odom_y0
            #calculate distance
            current_distance = self._set_odometry_total(odom_x0,odom_y0,odom_x1,odom_y1)
            
            
           
        else:
            #stop robot
            self.move_forward()   
        #rospy.loginfo("[move_forward_by_distance_follow_wall()]: --> exiting")     
    
    
    #Function moves the robot in a trajectory that attempts to keep the robot following a wall
    def move_follow_wall(self):
        rospy.loginfo("[move_follow_wall()]: --> started") 

        l_ranges = self.laser_subscriber.get_laser_ranges()
        len_ranges = len(l_ranges)
    
        front = int(len_ranges / 2 ) if len_ranges != 0 else 0
        #index= int( get_ray(90.0, l_ranges) ) 
        ray_90degrees = 80
        #loop_cnt, loop_cnt_max = 0, 1000
        #while True:
        #l_ranges = self.laser_subscriber.get_laser_ranges() # get laser readings. only ranges values
       
        if l_ranges[front] > self.wall_threshold:
            #Move straight
            if l_ranges[ray_90degrees] >= self.turn_left_threshold and l_ranges[ray_90degrees] <= self.turn_right_threshold:
                rospy.loginfo(f"[move_follow_wall()]: moving robot straight. distance from wall reading : {l_ranges[ray_90degrees]}")
                self.move_forward(self.x_speed, 0.0)
            #Turn left/away from wall. linear.x at half normal speed
            elif l_ranges[ray_90degrees] < self.turn_left_threshold:
                rospy.loginfo(f"[move_follow_wall()]: turning left. distance from wall reading : {l_ranges[ray_90degrees]}")
                self.move_forward(self.x_speed,self.z_speed)
                #Turn right/towards the wall . linear.x at half normal speed
            elif l_ranges[front] > self.turn_right_threshold :
                rospy.loginfo(f"[move_follow_wall()]: turning right. distance from wall reading : {l_ranges[ray_90degrees]}")
                self.move_forward(self.x_speed,-self.z_speed)
            else:
                    rospy.loginfo("[move_follow_wall()]: maintaining prev action: else not straight, not turn right/left ")
            
        else:
            #Robot close to the wall. turn left rapidly
            print(f"[move_follow_wall()]: Robot close to wall. turning left rapidly. front distance : {l_ranges[front]}")
            self.move_forward(self.x_speed/2, self.z_speed*10)
        
        
        rospy.loginfo("move_follow_wall(): exiting")  

    


