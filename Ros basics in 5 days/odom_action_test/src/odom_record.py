#! /usr/bin/env python
import rospy
import actionlib
from turtlebotMotionClass import turtlebotMotion
from odom_action_test.msg import OdomRecordAction, OdomRecordFeedback, OdomRecordResult
from tf_transformations import euler_from_quaternion, quaternion_from_euler

#action server class
class OdometryActionServer():
    def __init__(self):
        self._Odometry_action_server = actionlib.SimpleActionServer("record_odometry", OdomRecordAction, self.goal_callback, False)
        self._Odometry_action_server.start()
        self._turtlebot = turtlebotMotion()
        self._rate = rospy.Rate(1.0)
        self._Odometry_feedback = OdomRecordFeedback()
        self._Odometry_result = OdomRecordResult()
        self._Odometry_result.list_of_odoms =[] #empty list 
        self._Odometry_goal_success = False
        self._complete_lap_distance = 20.0 


    def goal_callback(self, Odom_goal):
        #get initial odometry values. Establish start point for calculating distance values
        odom_start_values = self._turtlebot.odometry_subscriber.get_odometry_data()

        while not self._Odometry_goal_success:
            #pre empting
            if self._Odometry_action_server.is_preempt_requested(): 
                rospy.loginfo("[goal_callback()]: goal is preempted")
                self._Odometry_goal_success= False
                self._Odometry_action_server.set_preempted()
                break

            #move_robot
            #self._turtlebot.move_follow_wall()

            self._turtlebot.move_forward_by_distance_follow_wall(odom_start_values,self._complete_lap_distance)

            #Update and publish feedback
            self._Odometry_feedback.current_total = self._turtlebot.odom_total_distance_x
            self._Odometry_action_server.publish_feedback(self._Odometry_feedback)
            self._rate.sleep()

            if self._Odometry_feedback.current_total >= self._complete_lap_distance:
                #stop robot 
                self._turtlebot.move_forward()
                #set success flag 
                self._Odometry_goal_success =True
                
        #action server is successful or prempted
        if self._Odometry_goal_success :
            #publish result 
            rospy.loginfo("[goal_ callback()]: goal is successful")
            self._Odometry_result.list_of_odoms = self._turtlebot.list_of_odoms[0]
            self._Odometry_action_server.set_succeeded(self._Odometry_result)


#############################
###    Main function      ###
#############################
def main():
    print("[main()]: started")
    #init action node
    rospy.init_node("turtlebot_action_server", anonymous=True, log_level=rospy.DEBUG)
    #init action server
    OdometryActionServer()

    execute_only_once= False
    while not rospy.is_shutdown():
        if not execute_only_once :
            rospy.loginfo("[]: while loop started")
            execute_only_once = True
        
    rospy.loginfo("[main()]: exiting")


if __name__  == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

