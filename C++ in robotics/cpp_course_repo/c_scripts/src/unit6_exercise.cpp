#include "ros/console.h"
#include "ros/duration.h"
#include "ros/time.h"
#include "rosbot_control/rosbot_class.h"
#include <ros/ros.h>
#include <string>
#include <iostream>
#include <math.h>
#include <list>

using namespace std;

class RobotMotion{
    private:
        float wall_threshold;
        float turn_right_threshold;
        float turn_left_threshold;
        int turn_time;
        RosbotClass robot{};
        double calculate_distance(double x0, double y0, double x1, double y1);
        

    public:
        
        RobotMotion();
        void move_robot();
        void move_robot_cafetaria();
        void move_robot_forward_distance(list<float> startOdometryValues,double distanceToTravel);

        
};

//Class functions
//Class constructor
RobotMotion::RobotMotion(){
    wall_threshold = 1.3;
    turn_right_threshold = 100.0;
    turn_left_threshold=0.8;
    turn_time = 2;   
}

//Function to move robot.
//trajectory defininition
// 1. move forward until close to wall
// 2. turn left
// 3. move forward until past right wall
// 4. turn right
// 5. move forward for short duration.
////////
void RobotMotion::move_robot(){
    float laser_front, laser_right;
    bool turn_right_allowed= false;
    while(true){
       
        laser_front=robot.get_laser(0);
        laser_right =robot.get_laser(200);
        ROS_INFO_STREAM("laser_front: " <<laser_front <<" laser_right: " <<laser_right);
        if( laser_front < wall_threshold && turn_right_allowed != true ){
            //Turn left
            robot.turn("counterclockwise", turn_time);
            robot.move_forward(4);
            turn_right_allowed= true;
        }
        else if (laser_right > turn_right_threshold  && turn_right_allowed ) {
            //turn right
            robot.turn("clockwise", turn_time);
            break;
        }
         //move forward
        robot.move_forward(1);

    }
    robot.move_forward(2);
    
}

//Function calculates distance between two odometry values
double RobotMotion::calculate_distance(double x0, double y0, double x1, double y1){
    ROS_INFO_STREAM("[calculate_distance()]: started");
    ROS_INFO_STREAM("[calculate_distance()]: x0="<<x0 <<" ,x1=" <<x1 <<" ,y0=" <<y0 <<" ,y1=" <<y1);
    double distance = sqrt( (x1-x0)*(x1-x0) + (y1-y0)*(y1-y0) );
    ROS_INFO_STREAM("[calculate_distance()]: distance: " <<distance);
    ROS_INFO_STREAM("[calculate_distance()]: exiting");
    return distance;
}

//Function moves robot forward for distance  
void RobotMotion::move_robot_forward_distance(list<float> startOdometryValues, double distanceToTravel ){
    ROS_INFO_STREAM("[move_robot_forward_distance()]: started");
    //Store start x and y odometry values
    auto iter = startOdometryValues.begin();//list iterator set to first element address
    double l_odom_x0= double( startOdometryValues.front() ); //first element
    double l_odom_y0= double(*++iter ); //increment iter to address of second element, store the value
    
    ROS_INFO_STREAM("[move_robot_forward_distance()]:l_odom_x0: " <<l_odom_x0 <<", l_odom_y0: "<<l_odom_y0);
    list<float> l_odom_x_y_z = startOdometryValues;
    iter = l_odom_x_y_z.begin();//list iterator set to first element address
    double l_odom_x1 = double( l_odom_x_y_z.front() );  //first element
    double l_odom_y1 = double( *++iter ); //increment iter to address of second element, store the value
    double currentDistanceTravelled = 0.0;
    int cnt = 0;
    int maxCnt= 50;
    ros::Time startTime = ros::Time::now();
    ros::Duration timeout(60.0); //4 secs


    //Move forward if distance is not reached
    ROS_INFO_STREAM("[move_robot_forward_distance()]: while loop started");
    while(  currentDistanceTravelled <= distanceToTravel ){
        
        //Move forward
        robot.move_forward(1);
        //get current odometry and store x and y values
        l_odom_x_y_z= robot.get_position_full(); 
        iter = l_odom_x_y_z.begin();
        l_odom_x1 = l_odom_x_y_z.front();
        l_odom_y1 = *++iter;
        currentDistanceTravelled = calculate_distance(l_odom_x0, l_odom_y0, l_odom_x1, l_odom_y1);
        ROS_INFO_STREAM("[move_robot_forward_distance()]: cnt: " <<cnt <<" ,currentDistanceTravelled: " <<currentDistanceTravelled <<" ,distanceToTravel: " <<distanceToTravel);
        ++cnt;
        
        if(cnt > maxCnt || ros::Time::now() - startTime > timeout ){ break;}
    }

    ROS_INFO_STREAM("[move_robot_forward_distance()]: exiting");

}

void RobotMotion::move_robot_cafetaria(){
    ROS_INFO_STREAM("[move_robot_cafetaria()]: started");
    robot.move_forward(3);
    robot.turn("clockwise",2); //Turn right
    robot.move_forward(8);
    robot.turn("counterclockwise",2); //Turn left
    move_robot_forward_distance(robot.get_position_full(), 7.7);
    ROS_INFO_STREAM("[move_robot_cafetaria()]: exiting");

}


int main(int argc, char** argv){
    ros::init(argc,argv,"Rosbot_move_node");
    
    //robot_motion object#include "rosbot_control/rosbot_class.h"
#include <ros/ros.h>
#include <string>
#include <iostream>
#include <math.h>
#include <list>

using namespace std;

class RobotMotion{
    private:
       
        float wall_threshold;
        float turn_right_threshold;
        float turn_left_threshold;
        int turn_time;
        list<float> l_odom_x_y_z;
        list<float> l_odom_x0_y0_z0;
        RosbotClass robot{};
        void calculate_distance();

    public:
        
        RobotMotion();
        void move_robot();
        void move_robot_cafetaria();
        void move_robot_forward_distance(float distance=5.0);

        
};

//Class funcitons
//class constructor
RobotMotion::RobotMotion(){
    wall_threshold = 1.3;
    turn_left_threshold=0.8;
    turn_right_threshold = 100.0;
    turn_time = 2;
    l_odom_x_y_z ={0,0,0}; //current odom
    l_odom_x0_y0_z0 = {0,0,0}; //previous odom
}

//Function to move robot.
//trajectory defininition
// 1. move forward until close to wall
// 2. turn left
// 3. move forward until past right wall
// 4. turn right
// 5. move forward for short duration.
////////
void RobotMotion::move_robot(){
    float laser_front, laser_right;
    bool turn_right_allowed= false;
    while(true){
       
        laser_front=robot.get_laser(0);
        laser_right =robot.get_laser(200);
        ROS_INFO_STREAM("laser_front: " <<laser_front <<" laser_right: " <<laser_right);
        if( laser_front < wall_threshold && turn_right_allowed != true ){
            //Turn left
            robot.turn("counterclockwise", turn_time);
            robot.move_forward(4);
            turn_right_allowed= true;
        }
        else if (laser_right > turn_right_threshold  && turn_right_allowed ) {
            //turn right
            robot.turn("clockwise", turn_time);
            break;
        }
         //move forward
        robot.move_forward(1);

    }
    robot.move_forward(2);
    
}

void RobotMotion::calculate_distance(){
//store to previous odom
for auto odom : l_odom_x_y_z:{
l_odom_x0_y0_z0[0] = l_odom_x_y_z[0];
l_odom_x0_y0_z0[1] = l_odom_x_y_z[1];
l_odom_x0_y0_z0[2] =l_odom_x_y_z[2];
}
//get position values
l_odom_x_y_z = robot.get_position_full();


}

void RobotMotion::move_robot_forward_distance(){

}

void RobotMotion::move_robot_cafetaria(){

}


    




int main(int argc, char** argv){
    ros::init(argc,argv,"Rosbot_move_node");
    
    //robot_motion object
    RobotMotion rosbot;
    rosbot.move_robot();
    return 0;
}
    RobotMotion rosbot;
    rosbot.move_robot_cafetaria();


     return 0;
}