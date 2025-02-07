#! /usr/bin/env python
import rospy
import actionlib
from ArdroneMotionClass import ArdroneMotion
#from actions_quiz.action import CustomActionMsgAction, CustomActionMsgFeedback 
from actions_quiz.msg import CustomActionMsgAction, CustomActionMsgFeedback, CustomActionMsgResult
 
class CustomActionServer():
    def __init__(self):
        #action server
        self._ardrone_action_server = actionlib.SimpleActionServer("action_custom_msg_as", CustomActionMsgAction, self.goal_callback, False)
        self._ardrone_action_server.start()
        self._ardrone_action_server_feedback = CustomActionMsgFeedback()
        #self._ardrone_action_server_result = CustomActionMsgResult()
        #drone motion object
        self._ardrone_bot = ArdroneMotion()
        self.rate = rospy.Rate(1)

    def goal_callback(self, ardrone_goal):
        rospy.loginfo("[goal_callback()]: started")
        ardrone_goal_success = False
        while not ardrone_goal_success:
            #pre empting
            if self._ardrone_action_server.is_preempt_requested(): 
                ardrone_goal_success = False
                self._ardrone_action_server.set_preempted()
                break
            
            # takeoff/land drone
            self._ardrone_bot._set_publisher_ardrone_topic(ardrone_goal.goal)
            # publish action feedback
            self._ardrone_action_server_feedback.feedback = ardrone_goal.goal
            self._ardrone_action_server.publish_feedback(self._ardrone_action_server_feedback)
            self.rate.sleep()

        #goal result
        #if ardrone_goal_success:
            #self._ardrone_action_server.set_succeeded()
        rospy.loginfo("[goal_callback()]: exiting")
            

###########################
###    main function    ###
###########################
def main():
    print("[main()]: started")
    execute_once_flag = False

    #init node
    rospy.init_node("ardrone_move_custom_action_server", anonymous=True)
    #service object
    CustomActionServer()

    while( not rospy.is_shutdown()):
       #execute once code
        if not execute_once_flag:
            rospy.loginfo("[main()]: while loop running")
            #Add code that needs to be executed once in here
            execute_once_flag = True
    
    rospy.loginfo("[main()]: exiting")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException: 
        print("Something aweful and fatal happened. Hope the bot is okay though.")
        print( ":-)" * 30 )
        pass

