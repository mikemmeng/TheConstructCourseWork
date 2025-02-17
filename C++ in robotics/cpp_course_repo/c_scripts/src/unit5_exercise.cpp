#include "ros/init.h"
#include "rosbot_control/rosbot_class.h"
#include <ros/ros.h>
using namespace std;

int main( int argc, char **argv){

    ros::init(argc,argv, "rosbot_node");

    RosbotClass rosbot;
    float laser_readings[2];
    float* laser_full=rosbot.get_laser_full();
    //move forward

    rosbot.move_forward(3);
    ROS_INFO_STREAM("laser full values: \n");
    //laser_readings[0] = rosbot.get_laser(700);
    //laser_readings[1] = rosbot.get_laser(20);
    //ROS_INFO_STREAM("Left reading: "  <<laser_readings[0]);
    //ROS_INFO_STREAM("Right reading: " <<laser_readings[1]);
    for (int cnt=0; cnt<720; cnt++) {
        ROS_INFO_STREAM(*laser_full);
        laser_full++;
    }
    
    return 0;
}
