import math, rospy
from utilities import spawn_coke_can, spawn_table, \
                      pause_physics, unpause_physics,\
                      set_model_state, get_model_state
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_multiply, \
                               quaternion_about_axis, \
                               quaternion_from_euler

unpause_physics()

axis_x = (1,0,0)
axis_y = (0,1,0) 
axis_z=  (0,0,1)
#table
position = Point(x=1,y=0,z=0)
roll, pitch, yaw = 0, 0, 90 
q = quaternion_from_euler(math.radians(roll), math.radians(pitch), math.radians(yaw))
#q_zt= quaternion_about_axis(math.radians(90),axis_z)
orientation= Quaternion(*q)
set_model_state('table', Pose(position, orientation))

#cans
position = Point(x=1,y=0,z=1.05)
for angle in range(0,360,30):
    cat = angle/30
    theta = math.radians(angle)
    xp = 0.2 * math.cos(theta) + 1
    yp = 0.2 * math.sin(theta)
    position = Point(xp, yp, 1.05)
    q_y  = quaternion_about_axis(math.radians(90), axis_y)
    q_z  = quaternion_about_axis(theta, axis_z)
    q_zy = quaternion_multiply(q_z, q_y)
    
    orientation= Quaternion(*q_zy)
    set_model_state('coke_can_'+str(cat), Pose(position, orientation))
    rospy.sleep(0.1)

    


        
        





    
