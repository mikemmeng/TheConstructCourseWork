#!/usr/bin/env python
import rospy
from bb8_move_circle_class  import MoveBB8
from my_python_class.srv import BB8CustomServiceMessage, BB8CustomServiceMessageResponse

def my_callback(request):
    rospy.loginfo("The Service move_bb8_in_circle has been called")
    bb8_move_service_response = BB8CustomServiceMessageResponse()
    movebb8_object = MoveBB8()
    
    movebb8_object.move_bb8(request.repetitions)
    rospy.loginfo("Finished service move_bb8_in_circle")
    bb8_move_service_response.success = True
    return bb8_move_service_response 

#callback = my_callback #test for assigning a function
rospy.init_node('service_move_bb8_in_circle_server', anonymous=True)
bb8_service = rospy.Service("/move_bb8_in_circle", BB8CustomServiceMessage, my_callback)
rospy.loginfo("Service /move_bb8_in_circle Ready")

rospy.spin() # keep the service open.
