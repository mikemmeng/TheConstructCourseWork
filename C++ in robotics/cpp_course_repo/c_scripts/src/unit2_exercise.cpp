
#include "rosbot_control/rosbot_class.h"
#include <iostream>
#include <ros/ros.h>
#include <list>

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "rosbot_node");

    RosbotClass rosbot;

    float coordinate_x0 = 0.0;
    float coordinate_x1 = 0.0;
    float x_limit= 0.0; 
    float goal_position = 10.0;
    list <float> coordinates;


    //get current position
    coordinate_x0 = rosbot.get_position(1);
    ROS_INFO_STREAM("coordinate_x0: " <<coordinate_x0 <<endl );
    // move robot forward
    rosbot.move_forward(5);
    coordinate_x1 = rosbot.get_position(1);
    coordinates = rosbot.get_position_full();
    ROS_INFO_STREAM("coordinate_x1: " <<coordinate_x1 <<endl);

    while(true){
        x_limit = coordinate_x1 - coordinate_x0;
        ROS_INFO_STREAM("coordinate_x0: " <<coordinate_x0 <<" coordinate_x1: " <<coordinate_x1 <<" x_limit: " <<x_limit <<endl );

        if(x_limit < goal_position){
            rosbot.move_forward(1);
            //update cooordinate 
            coordinate_x1 = rosbot.get_position(1);
        }
        else{
            ROS_INFO_STREAM("target position reached. stopping robot \n");
            rosbot.stop_moving();
            break;
        }
    }

    for (float coordinate : coordinates) {
        ROS_INFO_STREAM(coordinate <<",");
    
    } 
    cout <<endl;

  return 0; }