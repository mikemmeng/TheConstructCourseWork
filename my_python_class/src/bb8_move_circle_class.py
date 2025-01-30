#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class MoveBB8():
    
    def __init__(self):
        self.bb8_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd = Twist()
        self.ctrl_c = False
        self.rate = rospy.Rate(10) # 10hz
        rospy.on_shutdown(self.shutdownhook)
        
    def publish_once_in_cmd_vel(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuous publishing systems, this is no big deal, but in systems that publish only
        once, it IS very important.
        """
        while not self.ctrl_c:
            connections = self.bb8_vel_publisher.get_num_connections()
            if connections > 0:
                self.bb8_vel_publisher.publish(self.cmd)
                rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()
   
    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True

    def set_speed(self, x_speed=0.0,z_speed=0.0):
        self.cmd.linear.x= x_speed
        self.cmd.linear.y=0.0
        self.cmd.linear.z= 0.0
        self.cmd.angular.x = 0.0
        self.cmd.angular.y = 0.0
        self.cmd.angular.z = z_speed

    def move_bb8(self, move_repetitions=1, linear_speed=0.2, angular_speed=0.2):
        
        self.set_speed(linear_speed,angular_speed)
        move_repetitions_cnt = 0
        rospy.loginfo(f'Moving BB8! {move_repetitions} times')
        while not self.ctrl_c and move_repetitions_cnt <= move_repetitions:
            self.publish_once_in_cmd_vel()
            self.rate.sleep()
            move_repetitions_cnt+=1

        #Stop bb8
        self.set_speed()
        self.publish_once_in_cmd_vel()  

if __name__ == '__main__':
    rospy.init_node('move_bb8_test', anonymous=True)
    movebb8_object = MoveBB8()
    try:
        movebb8_object.move_bb8()
    except rospy.ROSInterruptException:
        pass