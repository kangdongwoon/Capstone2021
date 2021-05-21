#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

static geometry_msgs::Twist sim_msg;
static geometry_msgs::Twist carvel;
static bool reverse;
static bool start;
void CarvelCallback(const geometry_msgs::Twist::ConstPtr &vel){
    carvel.linear.x = vel->linear.x;
}
void sim_callback(const sensor_msgs::Joy::ConstPtr &msg)
{
  /*axes[0] == wheel, axes[2] == break, axes[5] = accel*/
  if((double)msg->buttons[0] == 1.){
      start = !start;
  }
  if(start){
      std::cout<<"start"<<std::endl;
        if((double)msg->axes[2] < -0.5 && (double)msg->buttons[3] == 1.){
            reverse = !reverse;
        }
        /*후진*/
        if(reverse){
            if((double)msg->axes[2] < 0.9){ //Break ON
              sim_msg.linear.x = -(((double)msg->axes[2])*3 / 40. + 0.075);
            }
            else{                           //Acceleration ON
                sim_msg.linear.x = ((double)msg->axes[5] / 20. - 0.20); //후진 최고속도 : 0.25m/s 기본속도 : 0.15m/s
            }
          sim_msg.angular.z = (double)msg->axes[0] * 0.576; // -1 ~ 1 -> -? ~ ?
          // Simulation input : joystick/cmd_vel
          // Steering input : /cmd_vel
        }
        /*전진*/
        else{
            if((double)msg->axes[2] < 0.9){ //Break ON
              sim_msg.linear.x = (((double)msg->axes[2])*3. / 40. + 0.075);
            }
            else{                           //Acceleration ON
              sim_msg.linear.x = (((double)msg->axes[5])*27. / -40. + 0.825);//전진 최고속도 : 1.5m/s 기본속도 : 0.15m/s
            }
            sim_msg.angular.z = (double)msg->axes[0] * 0.576; //sh ->0.9
        }
  }
  else {
      sim_msg.linear.x = 0.0;
      sim_msg.angular.z = 0.0;
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sim_node");
  ros::NodeHandle nh;

  ros::Subscriber sim_sub = nh.subscribe("/controller/gui_command",1, &sim_callback);
  ros::Subscriber car_vel = nh.subscribe("/des_vel/cmd_vel",1, &CarvelCallback);
  ros::Publisher sim_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
//  ros::Publisher sim_pub = nh.advertise<geometry_msgs::Twist>("/joystick/cmd_vel", 100);
  // Simulation input : /des_vel/cmd_vel
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
