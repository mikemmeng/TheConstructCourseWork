#! /usr/bin/env python

import rospy
from ardrone_as.msg import ArdroneGoal, ArdroneFeedback, ArdroneResult
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty


class ArdroneMotion():
    def __init__(self):
       #publishers
        self.ardrone_move_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.ardrone_takeoff_publisher = rospy.Publisher("/drone/takeoff", Empty, queue_size=1)
        self.ardrone_land_publisher = rospy.Publisher("/drone/land", Empty, queue_size=1)
        self.ardrone_move = Twist()
        self.ardrone_empty_msg = Empty()
        self.rate = rospy.Rate(1.0)
        self.move_square_current_side = 0
        self.move_square_total_time_taken = 0
    
    

    #sets the twist msg variables
    def _set_speed(self, linear_x_speed=0.0,linear_y_speed=0.0,angular_z_speed=0.0):
        self.ardrone_move.linear.x = linear_x_speed
        self.ardrone_move.linear.y = linear_y_speed
        self.ardrone_move.linear.z = 0.0
        self.ardrone_move.angular.x = 0.0
        self.ardrone_move.angular.y = 0.0
        self.ardrone_move.angular.z = angular_z_speed
    
    #Function publishes to cmd_vel topic.
    def _set_publisher_cmd_vel(self):
        #check publisher connection and publish msg
        #rospy.loginfo("[_set_publisher_cmd_vel()]: started. publishing cmd_vel topic msg")
        while self.ardrone_move_publisher.get_num_connections() < 1:
            pass
        self.ardrone_move_publisher.publish(self.ardrone_move)
        #rospy.loginfo("[_set_publisher_cmd_vel()]: exiting. publishing cmd_vel topic msg successful")

    #Function publishes to ardrone topics
    def _set_publisher_ardrone_topic(self, topic="LAND"):
        
        #Check publisher connection and publish msg
        rospy.loginfo("[_set_publisher_ardrone_topic()]: started. publishing cmd_vel topic msg")
        
        #Land topic
        if( topic.upper() == "LAND"):
            rospy.loginfo("[_set_publisher_ardrone_topic()]: land publisher checking and establishing connection.")
            while self.ardrone_land_publisher.get_num_connections() < 1:
                pass
            self.ardrone_land_publisher.publish(self.ardrone_empty_msg)
            rospy.loginfo("[_set_publisher_ardrone_topic()]: land publisher published successfully.")

        #Takeoff topic
        elif(topic.upper() == "TAKEOFF" ):
            rospy.loginfo("[_set_publisher_ardrone_topic()]: takeoff publisher checking and establishing connection.")
            while self.ardrone_takeoff_publisher.get_num_connections() < 1:
                pass
            self.ardrone_takeoff_publisher.publish(self.ardrone_empty_msg)
            rospy.loginfo("[_set_publisher_ardrone_topic()]: takeoff publisher published successfully.")
        else: 
            pass
        
        #
        rospy.loginfo("[_set_publisher_ardrone_topic()]: exiting. publishing cmd_vel topic msg successful")
    
    #Function to move the ardrone
    def move(self, x_speed=0.0, y_speed=0.0,z_speed=0.0):
        rospy.loginfo("[move()] : started")
        self._set_speed(x_speed,y_speed,z_speed)
        rospy.loginfo(f"[move()] : x_speed: {x_speed}, y_speed:{y_speed}, z_speed:{z_speed}")
        self._set_publisher_cmd_vel()
        rospy.loginfo("[move())] : exiting")
    #Function to move forward/reverse ie along x-axis of drone
    # To reverse use a negative value for x_speed 
    def move_forward(self,x_speed=0.15,side=3):
        rospy.loginfo("[move_forward()]: started")
        cnt_side = 0
        #move forward by given side/time
        while cnt_side < side:
            rospy.loginfo(f"[move_forward()]: cnt_side: {cnt_side}")
            self.move(x_speed,0.0,0.0)
            self.rate.sleep()
            cnt_side+=1 
        #stop robot
        self.move() #defualt paramters are 0.0. Will stop the robot.
        rospy.loginfo("[move_forward()]: exiting")
    #Function to move right/left ie along y-axis of drone
    #To move left use a negative value for y_speed 
    def move_right(self,y_speed=0.15,side=3):
        rospy.loginfo("[move_right()]: started")
        cnt_side = 0
        #move forward by given side/time
        while cnt_side < side:
            rospy.loginfo(f"[move_right()]: cnt_side: {cnt_side}")
            self.move(0.0,y_speed,0.0)
            self.rate.sleep()
            cnt_side+=1 
        #stop robot
        self.move() #defualt paramters are 0.0. Will stop the robot.
        rospy.loginfo("[move_right()]: exiting")

    #Function to move up/down ie along z-axis of drone
    #To move down use a negative value for z_speed 
    def move_up(self,z_speed=0.15,side=3):
        rospy.loginfo("[move_up()]: started")
        cnt_side = 0
        #move forward by given side/time
        while cnt_side < side:
            rospy.loginfo(f"[move_up()]: cnt_side: {cnt_side}")
            self.move(0.0,0.0,z_speed)
            self.rate.sleep()
            cnt_side+=1 
        #stop robot
        self.move() #defualt paramters are 0.0. Will stop the robot.
        rospy.loginfo("[move_up()]: exiting")

    #Function to move the ardrone in a square
    def move_square(self,x_speed=0.15,y_speed=0.15, side=3): 
        rospy.loginfo(f"[move_square()]: started")       
        #Step 1: takeoff
        self._set_publisher_ardrone_topic("TAKEOFF")
        distance = side
        turns= 1
        start_time=rospy.get_time()
        while turns <= 2:
            rospy.loginfo(f"[move_square()]: turn : {turns}")
            self.move_square_current_side=turns
            #Change direction from forward/right to reverse/left by inverting the sign of speed value
            if turns == 2:
                rospy.loginfo("[move_square()]: change direction")
                x_speed *=-1
                y_speed *=-1
            self.move_forward(x_speed,distance)
            self.move_right(y_speed, distance)
            turns+=1
        end_time=rospy.get_time()
        self.move_square_total_time_taken = end_time - start_time
        rospy.loginfo("[move_square()]: exiting")
        
        
        
     

        


