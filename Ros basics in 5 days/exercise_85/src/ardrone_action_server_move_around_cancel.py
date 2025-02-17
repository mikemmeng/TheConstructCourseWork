#! /usr/bin/env python

import rospy
import actionlib
import time
from ardrone_as.msg import ArdroneAction, ArdroneGoal,ArdroneResult, ArdroneFeedback 
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
#callbacks
"""
class SimpleGoalState:
    PENDING = 0
    ACTIVE = 1
    DONE = 2
    WARN = 3
    ERROR = 4

"""
# We create some constants with the corresponing vaules from the SimpleGoalState class
PENDING = 0
ACTIVE = 1
DONE = 2
WARN = 3
ERROR = 4

nImage = 1
# definition of the feedback callback. This will be called when feedback
# is July 25, 5 PM CEST!received from the action server
# it just prints a message indicating a new message has been received
def feedback_callback(feedback):
    global nImage
    print(f'[Feedback] image n.{nImage} received')
    nImage += 1


#### inits ####
#Node
rospy.init_node("ardrone_action_client_move_around", anonymous=True)
#Action client
ardrone_client = actionlib.SimpleActionClient("/ardrone_action_server", ArdroneAction)
ardrone_client.wait_for_server()

#Publisher 
ardrone_move_publisher = rospy.Publisher("/cmd_vel",Twist, queue_size=1)
ardrone_takeoff_publisher = rospy.Publisher("/drone/takeoff", Empty, queue_size=1)
ardrone_land_publisher = rospy.Publisher("/drone/land", Empty, queue_size=1)
ardrone_move = Twist()
ardrone_empty_msg = Empty()

rate= rospy.Rate(1)

#Create and launch action goal
ardrone_goal = ArdroneGoal()
ardrone_goal.nseconds = 12
ardrone_client.send_goal(ardrone_goal,feedback_cb=feedback_callback)

#### function definitions ####

# sets the twist msg variables
def set_speed(linear_x_speed=0.0,linear_y_speed=0.0,angular_z_speed=0.0):
    ardrone_move.linear.x = linear_x_speed
    ardrone_move.linear.y = linear_y_speed
    ardrone_move.linear.z = 0.0
    ardrone_move.angular.x = 0.0
    ardrone_move.angular.y = 0.0
    ardrone_move.angular.z = angular_z_speed

# publish to cmd_vel
def move_around(x_speed=0.0, y_speed=0.0,z_speed=0.0):
    rospy.loginfo("[move_around()] : started")
    set_speed(x_speed,y_speed,z_speed)
   #check publisher connection and publish msg
    rospy.loginfo("[move_around()]: publishing move msg")
    while ardrone_move_publisher.get_num_connections() < 1:
        pass
    ardrone_move_publisher.publish(ardrone_move)
    rospy.loginfo("[move_around()]: publishing move msg completed")

    rospy.loginfo("[move_around()] : exiting")


def main():
    rospy.loginfo("[main()]: started")
    #take_off the drone
    rospy.loginfo("[main()]: publishing takeoff msg")
    while ardrone_takeoff_publisher.get_num_connections() < 1:
        pass
    ardrone_takeoff_publisher.publish(ardrone_empty_msg)
    rospy.loginfo("[main()]: publishing takeoff msg successful.")
    
    
    #get action server state
    ardrone_state_result = ardrone_client.get_state()
    cancel_cnt = 0
    while ardrone_state_result < DONE:
        #rospy.loginfo("Doing stuff while waiting for the action server to give a result....")
        move_around(0.15,0.1,0.0) # move forward, move up
        rate.sleep()
        ardrone_state_result = ardrone_client.get_state()
        rospy.loginfo(f'ardrone_state_result: {ardrone_state_result}')
        if cancel_cnt > 4:
            rospy.logwarn(f'[main()]: cancel_cnt:{cancel_cnt} threshold reached. cancelling goal')
            ardrone_client.cancel_goal()
            rospy.logwarn("[main()]: goal cancelled")
            break
        cancel_cnt+=1
    #land drone
    rospy.loginfo("[main()]: publishing land msg")
    while ardrone_land_publisher.get_num_connections() < 1:
        pass
    ardrone_land_publisher.publish(ardrone_empty_msg)
    rospy.loginfo("[main()]: publishing land msg successful")
    
    
    rospy.loginfo(f'[Result] state: {ardrone_state_result}')
    if ardrone_state_result == ERROR:
        rospy.logerr("Something went wrong in the server Side")
    if ardrone_state_result == WARN:
        rospy.logwarn("There is a warning in the server Side")
    
    rospy.loginfo("[main()]: exiting")

if __name__ == '__main__': 
    try: 
        main() 
    except rospy.ROSInterruptException: 
        print("Something terrible has happened...Hope the drone is okay")
        print(":-)" * 30)  
        

