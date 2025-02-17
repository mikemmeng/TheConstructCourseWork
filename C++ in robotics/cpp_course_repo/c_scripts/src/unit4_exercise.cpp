
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

string move_trajectory(RosbotClass rosbot, int n_secs) {

  string direction = "clockwise";
  string result = "Trajectory successful";
  rosbot.move_forward(n_secs);
  rosbot.turn(direction, n_secs);
  rosbot.move_forward(n_secs);
  return result;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "rosbot_node");

  RosbotClass rosbot;
  string result;
  // list<float> coordinates;
  // coordinates = move_and_inform(rosbot, 5);
  // string trajectory = move_trajectory(rosbot, 5);
  // ROS_INFO(coordinates[0] << ", " coordinates[1] << endl);
  result = move_trajectory(rosbot,5);
  ROS_INFO_STREAM(result << endl);

  return 0;
}