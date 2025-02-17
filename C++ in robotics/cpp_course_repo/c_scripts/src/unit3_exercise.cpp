
#include "rosbot_control/rosbot_class.h"
#include <iostream>
#include <list>
#include <ros/ros.h>
#include <string>

using namespace std;

// functions
list<float> move_and_inform(RosbotClass rosbot, int n_secs) {

  // Move robot
  rosbot.move_forward(n_secs);
  // get x and y coordinates
  float x_coordinate = rosbot.get_position(1);
  float y_coordinate = rosbot.get_position(2);
  list<float> coordinates({x_coordinate, y_coordinate});
  return coordinates;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "rosbot_node");

  RosbotClass rosbot;
  list<float> coordinates;
  coordinates = move_and_inform(rosbot, 5);
  // string trajectory = move_trajectory(rosbot, 5);
  //ROS_INFO_STREAM(coordinates <<endl);

  return 0;
}