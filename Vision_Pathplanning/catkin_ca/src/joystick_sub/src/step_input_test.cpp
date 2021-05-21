#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include "wrp_sdk/platforms/hunter/hunter_base.hpp"
#include "hunter_msgs/HunterStatus.h"

using namespace std;

static geometry_msgs::Twist car_vel;
static sensor_msgs::Joy joy;
static hunter_msgs::HunterStatus hunter;

static double angular = 0.1;
static double MAX = 0.576;
void JoystickCallback(const sensor_msgs::Joy::ConstPtr &msg){
    if(msg->buttons[0] == 1){
        car_vel.angular.z += angular;
    }
    if(msg->buttons[2] == 1){
        car_vel.angular.z -= angular;
    }
    if(msg->buttons[1] == 1){
        car_vel.angular.z = 0;
    }
    if(msg->buttons[3] == 1){
        MAX = -MAX;
        car_vel.angular.z = MAX;
    }
    cout << "angular\t" << car_vel.angular.z << endl;
    cout << "h angle\t" << hunter.steering_angle << endl << endl;
    car_vel.linear.x = -0.1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "step_input_test");
    ros::NodeHandle nh;
    ros::Subscriber joystick = nh.subscribe("/controller/gui_command",1, &JoystickCallback);
//    ros::Publisher vel_input = nh.advertise<geometry_msgs::Twist>("/test/cmd_vel", 100);
    ros::Publisher vel_input = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    ros::Rate rate(50);
    while (ros::ok())
    {
      vel_input.publish(car_vel);
      rate.sleep();
      ros::spinOnce();
    }
}
