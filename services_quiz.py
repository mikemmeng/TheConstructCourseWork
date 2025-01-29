#! /usr/bin/env python

import rospy
from services_quiz.srv import BB8CustomServiceMessage, BB8CustomServiceMessageResponse
from geometry_msgs.msg import Twist

#Robot motion class
#implements bb8 robot motions via a service
class robot_motion:
    def __init__(self):
        #init_service
        rospy.init_node('move_bb8_in_square_custom_server') 
        self.move_service = rospy.Service('/move_bb8_in_square_custom',BB8CustomServiceMessage , self.callback) 
        #init publisher
        self.move_pub1 = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.move= Twist()
        #class attributes
        self.x_speed = 0.35
        self.z_speed = 0.52
        self.move_side = 0.0
        self.move_repetition = 0
        self.move_response = False
       

    #Function to initialize the service
    def start_service(self):
        #init service
        rospy.init_node('move_bb8_in_square_custom_server') 
        self.move_service = rospy.Service('/move_bb8_in_square_custom',BB8CustomServiceMessage , self.callback) 

    #Function callback. Service callback function
    def callback(self, request):
        rospy.loginfo("Function callback(): The service move_bb8_in_square_custom has been called")
        
        self.move_side = request.side
        print(f'function callback(): self.move_side : {self.move_side}')
        self.move_repetition = request.repetitions
        print(f'function callback(): self.move_repetitions : {self.move_repetition}')
        self.move_response = BB8CustomServiceMessageResponse()   
        print(f'function callback(): self.move_response : {self.move_response}')
       
        #Service function calls
        self.move_squares(self.x_speed, self.z_speed, self.move_side, self.move_repetition )
       
        rospy.loginfo("Function callback(): finished service move_bb8_in_square")
        return self.move_response 

    # Function to set speed. 
    # Only linear.x and angular.z from twist msg required to control motion
    # default args are set to zero to stop robot
    def _set_speed(self, x_speed=0.0, z_speed=0.0):
        self.move.linear.x = x_speed
        self.move.linear.y = 0.0
        self.move.linear.z = 0.0
        self.move.angular.x = 0.0
        self.move.angular.y = 0.0
        self.move.angular.z = z_speed
        

    #Function moves robot in a straight line at set speed and distance
    # sleep rate is 1 sec
    def move_straight(self, x_speed=0.3, side=3.0):
        rospy.loginfo("Function move_straight():--> Started")
        distance = side
        rospy.loginfo(f'Function move_straight(): distance to move {distance}')
        rate = rospy.Rate(1) #1 sec
        if distance > 0:    
            #move straight for specified duration  
            while True:
                rospy.loginfo(f'Function move_straight(). Moving straight. Distance remaining: {distance}')
                self._set_speed(x_speed,0.0)
                self.move_pub1.publish(self.move)
                rate.sleep()
                distance -=1.0 #reduce distance by 1 
                if distance <= 0.0:
                    #stop robot
                    rospy.loginfo(f'Function move_straight(). Stopping bb8.')
                    self._set_speed() # default value for stop
                    self.move_pub1.publish(self.move)
                    rate.sleep()
                    break #exit when distance is zero
            self.move_response.success = True
        else: 
            self.move_response.success = False

        rospy.loginfo("Function move_straight():--> exiting")
      
   #Function turns the robot for the given distance
   # aim is not to achieve perfect turn eg 90degrees, 60degrees but to turn 
   # sleep rate is 1 sec
    def move_turn(self, z_speed=0.5, side=3.0):
        rospy.loginfo("Function move_turn():--> Started")
        distance = side
        rospy.loginfo(f'Function move_turn(): distance to move {distance}')
        rate = rospy.Rate(1) #1 sec
        if distance > 0:    
            #turn for specified duration  
            while True:
                rospy.loginfo(f'Function move_turn(). Turning . Distance remaining: {distance}')
                self._set_speed(0.0,z_speed)
                self.move_pub1.publish(self.move)
                rate.sleep()
                distance -=1.0 #reduce distance by 1 
                if distance <= 0.0:
                    #stop robot
                    rospy.loginfo(f'Function move_turn(). Stopping bb8.')
                    self._set_speed() # default value for stop
                    self.move_pub1.publish(self.move)
                    rate.sleep()
                    break #exit when distance is zero
            self.move_response.success = True
        else: 
            self.move_response.success = False

        rospy.loginfo("Function move_turn():--> exiting")

   #Function moves the robot in a square like movement
   # may not achieve perfect square motion, so long as it is square like    
    def move_square(self, x_speed=0.3, z_speed=0.5, side=3.0):
        rospy.loginfo("Function move_square():--> Started")
        distance = side
        distance_square = 3.0 #Fixed value
        rospy.loginfo(f'Function move_square(): distance to move {distance}')
        turns= 1
        #to achieve square movements
        # straight 4 times
        # right turns 4 times
        while turns <= 4:
            rospy.loginfo(f'Function move_square(): turn : {turns}')
            self.move_straight(x_speed,distance)
            self.move_turn(z_speed,distance_square)
            turns+=1
        rospy.loginfo("Function move_square():--> exiting")

    #Function turns the robot in square like motion 
    #and repeats the motion given number of times
    def move_squares(self,x_speed=0.3, z_speed=0.3, side=3.0, repetitions=0):
        rospy.loginfo("Function move_squares(): --> started")
        repeat = repetitions
        while repeat >= 0:
            rospy.loginfo(f'Function move_squares(): calling move_square(). repeat: {repeat}')
            self.move_square(x_speed,z_speed,side)
            repeat-=1
        rospy.loginfo("Function move_squares(): --> exiting")

###################################
#                                 #
#Program main                     #
#                                 #
###################################
def main():
    print(" function main(): started ")
    rospy.loginfo("Service /move_bb8_in_circle_custom ready")
    bb8 = robot_motion()
    
    main_print_once = False
    while not rospy.is_shutdown(): 
       if not main_print_once: 
            print(" function main(): while loop. Press CTRL+C to exit")
            main_print_once = True
       
      
if __name__ == '__main__': 
    try: 
        main() 
    except rospy.ROSInterruptException: 
        print("Something aweful and fatal happened. Hope the bot is okay though.")
        print( ":-)" * 30 )
        pass



