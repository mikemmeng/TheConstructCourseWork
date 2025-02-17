#include <string>
#include <list>
#include <iostream>
#include <map>

#include "rosbot_control/rosbot_class.h"
#include <ros/ros.h>


using namespace std;

int main(int argc, char **argv) {
  ros::init(argc, argv, "rosbot_node");

  RosbotClass rosbot;
 
  map<double, double> coordinate_dictionary;
 
  rosbot.move();
  double coordinate_x0 = rosbot.get_position(1);
  double timestamp_t0 = rosbot.get_time();
  coordinate_dictionary[timestamp_t0]= coordinate_x0;
  cout << "coordinate : " << coordinate_x0 << " timestamp : " << timestamp_t0 << "\n" ;
  
  rosbot.move();
  double coordinate_x1 = rosbot.get_position(1);
  double timestamp_t1 = rosbot.get_time();
  coordinate_dictionary[timestamp_t1]= coordinate_x1;
  cout << "coordinate : " << coordinate_x1 << " timestamp : " << timestamp_t1 << "\n" ;
 
  for ( auto coordinate : coordinate_dictionary)
       cout << coordinate.first << " : " << coordinate.second <<"\n" ; 

  double velocity = (coordinate_x1 - coordinate_x0) / (timestamp_t1 - timestamp_t0);
 
  if(velocity < 1.00 )
     ROS_INFO_STREAM("true"); 
   
  
    


  return 0;
}