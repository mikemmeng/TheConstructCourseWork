#! /usr/bin/env python

import rospy
from ArdroneMotionClass import ArdroneMotion
#tests for ArdoneMotion class



### Main function ###
def main():
    rospy.loginfo("[main()]: started")
    execute_once_flag = False
    #init node
    rospy.init_node("ardrone_action_move_square") 
    
    #init ArdroneMotinObject
    ardrone_test =ArdroneMotion() 
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        if not execute_once_flag:
            rospy.loginfo("[main()]: while loop started")
            #test _set_publisher_ardrone_topic()
            #ardrone_test._set_publisher_ardrone_topic("TAKEOFF")
            #test move() function
            #ardrone_test.move(0.15,0.15,0.15)
            #test move_forward() function
            #ardrone_test.move_forward(-0.2, 4)
            #test move_right() function
            #ardrone_test.move_right(0.2, 10)
            #test move_up() function
            #ardrone_test.move_up(0.2, 10)
            #test move_square() function
            ardrone_test.move_square(0.2,0.2,4)
            #cnt=0
            #while cnt < 10:
            #    rospy.loginfo(f"[main()]: cnt: {cnt}")
            #    rate.sleep()
            #    cnt+=1
            #test _set_publisher_ardrone_topic()
            #ardrone_test._set_publisher_ardrone_topic("LAND")
            execute_once_flag= True


        
    rospy.loginfo("[main()]: exiting")

if __name__ == "__main__":
    try: 
        main() 
    except rospy.ROSInterruptException: 
        print("Something aweful and fatal happened. Hope the bot is okay though.")
        print( ":-)" * 30 )
        pass
  