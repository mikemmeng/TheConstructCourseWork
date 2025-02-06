#! /usr/bin/env python

import rospy
import actionlib
from actionlib.msg import TestAction, TestFeedback, TestResult
from ArdroneMotionClass import ArdroneMotion

class ArdroneServer():
    def __init__(self):
        #Create the action server
        self._ardrone_server = actionlib.SimpleActionServer("ardrone_action_server", TestAction, self.goal_callback, False)
        self._ardrone_server.start()
        #robot motion object
        self._ardrone_bot= ArdroneMotion()
        #Server feedback and result
        self._feedback = TestFeedback()
        self._result = TestResult()
           
    def goal_callback(self, ArdroneGoal):
            
        ArdroneSuccess = False
        while not ArdroneSuccess:
            #preempted check
            if self._ardrone_server.is_preempt_requested():
                rospy.loginfo("[goal_feedback()]: goal is preempted")
                self._ardrone_server.set_preempted()
                ArdroneSuccess=False
                break
            #update feedback
            self._feedback.feedback = self._ardrone_bot.move_square_current_side
            self._ardrone_server.publish_feedback(self._feedback)
            #move bot
            self._ardrone_bot.move_square(0.15,0.15, ArdroneGoal.goal)
            #Set success flag
            ArdroneSuccess= True
                
            
        #Robot moved successfuly or was preempted
        if ArdroneSuccess:
            #publish result
            rospy.loginfo("[goal_feedback()]: goal is successful")
            self._result.result = int( self._ardrone_bot.move_square_total_time_taken )
            self._ardrone_server.set_succeeded(self._result)
            #land drone
            self._ardrone_bot._set_publisher_ardrone_topic("LAND")


#####################
### Main function ###
#####################

def main():
    rospy.loginfo("[main()]: started")
    execute_once_flag = False
    #init node
    rospy.init_node("ardrone_action_move_square", anonymous=True) 
    #init service
    ArdroneServer() #init service

    while not rospy.is_shutdown():
        if not execute_once_flag:
            rospy.loginfo("[main()]: while loop started")
            execute_once_flag= True
        
    rospy.loginfo("[main()]: exiting")


if __name__ == "__main__":
    try: 
        main() 
    except rospy.ROSInterruptException: 
        print("Something aweful and fatal happened. Hope the bot is okay though.")
        print( ":-)" * 30 )
        pass
  




        


