#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>


geometry_msgs::Twist sim_msg;
bool reverse;
void sim_callback(const sensor_msgs::Joy::ConstPtr &msg)
{
  if((double)msg->axes[5] < 0.5)
      if((double)msg->buttons[3] == 1.)
        reverse = !reverse;
  std::cout << reverse << std::endl;
  if(reverse){ //axes[4] : 0.
//    sim_msg.linear.x = ((double)msg->axes[4] / 4. - 0.25); //+ ((double)msg->axes[5] / 2. - 0.5);
    sim_msg.linear.x = -0.4;
    sim_msg.angular.z = (double)msg->axes[0] * -0.4;
    // Simulation input : joystick/cmd_vel
    // Steering input : /cmd_vel
  }
  else{
//    sim_msg.linear.x = ((double)msg->axes[4] / -4. + 0.25);
    sim_msg.linear.x = 0.4;
    sim_msg.angular.z = (double)msg->axes[0] * 0.4;
  }

  std::cout<<"msg->0"<<(double)msg->axes[0]<<std::endl;
  std::cout<<"linear: "<<sim_msg.linear.x<<" angular: "<<sim_msg.angular.z<<std::endl;


  // CONST CAR Velocity
//  sim_msg.linear.x = -0.4;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sim_node");
  ros::NodeHandle nh;

  ros::Subscriber sim_sub = nh.subscribe("/controller/gui_command",1, &sim_callback);
  ros::Publisher sim_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  // Simulation input : joystick/cmd_vel
  // Steering input : /cmd_vel
  ros::Rate rate(50);
  while (ros::ok())
  {
    sim_pub.publish(sim_msg);
    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
