#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "gazebo_msgs/ModelStates.h"
#include "sensor_msgs/Imu.h"
#include "iostream"
#include "pose_msgs/msgAng.h"
#include "pose_msgs/msgPose.h"

#include "cstdlib"
#include "ctime"

#define SQR(x) ((x)*(x))
#define PI  3.1415926535

using namespace std;

static geometry_msgs::Pose CarPose;
static geometry_msgs::Pose TrailerPose;
static geometry_msgs::Pose LLink;
static geometry_msgs::Pose RLink;
static geometry_msgs::Twist CarVelo;
static geometry_msgs::Twist TrailerVelo;

static pose_msgs::msgPose TrailerAng;

static double Vehicle_Yaw[3];
static double Dcar = 0.4216;
static double Dtrailer = 0.4230;

static double L = 0.650;
static double pre_data = 0;
static double dt = 0.01;
double LPF(double input, double tau){
  cout<<"pre_data:"<<pre_data<<endl;
  pre_data = ((tau * pre_data) + (dt * input)) / (tau + dt);
  return pre_data;
}
void PosErrorCallBack(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  // Angle BTW Car & Trailer(Theta) : Vehicle_Yaw[2] * 180 / PI
  CarPose.orientation = msg->pose[1].orientation; //base_link
  TrailerPose.orientation = msg->pose[6].orientation; //potential_link

  // CarPos Eular Yaw
  Vehicle_Yaw[0] = (double)(atan2f(2.0*(CarPose.orientation.w*CarPose.orientation.z - CarPose.orientation.x*CarPose.orientation.y),
      -SQR(CarPose.orientation.x)+SQR(CarPose.orientation.y)-SQR(CarPose.orientation.z)+SQR(CarPose.orientation.w)));
  // Trailer Eular Yaw
  Vehicle_Yaw[1] = (double)(atan2f(2.0*(TrailerPose.orientation.w*TrailerPose.orientation.z - TrailerPose.orientation.x*TrailerPose.orientation.y),
      -SQR(TrailerPose.orientation.x)+SQR(TrailerPose.orientation.y)-SQR(TrailerPose.orientation.z)+SQR(TrailerPose.orientation.w)));
  Vehicle_Yaw[2] = Vehicle_Yaw[0] - Vehicle_Yaw[1];
  TrailerAng.trailer_yaw = Vehicle_Yaw[2];

  if(Vehicle_Yaw[2] >= PI) Vehicle_Yaw[2] -= PI;
  if(Vehicle_Yaw[2] <= PI) Vehicle_Yaw[2] += PI;

  // Position Car : Rear Wheels Link Average(x,y)
  CarPose.position.x = (msg->pose[3].position.x + msg->pose[5].position.x)/2.0;
  CarPose.position.y = (msg->pose[3].position.y + msg->pose[5].position.y)/2.0;

  // Position Trailer
  TrailerPose.position.x = (msg->pose[7].position.x + msg->pose[8].position.x)/2.0;
  TrailerPose.position.y = (msg->pose[7].position.y + msg->pose[8].position.y)/2.0;

  // Dcar Measure
  Dcar = sqrt(SQR(msg->pose[6].position.x - CarPose.position.x)+SQR(msg->pose[6].position.y - CarPose.position.y));
  // Dtrailer Measure
  Dtrailer = sqrt(SQR(msg->pose[6].position.x-TrailerPose.position.x)+SQR(msg->pose[6].position.y-TrailerPose.position.y));
}

static double CarVelMtx[2];//v0 w0
static double TrailerVelMtx[2];//v1 w1
static double KinematicsMtx[4];

static double kp = 3;
static double gazebo_angle = 0.;
void VeloKinematicsCallback(const geometry_msgs::Twist::ConstPtr& msg){
  TrailerVelo = *msg;
  TrailerVelMtx[0] = TrailerVelo.linear.x;
  TrailerVelMtx[1] = TrailerVelo.angular.z;

  /* dCar dTrailer Position Changed by Kinematics Perspective Change in System+ */
  KinematicsMtx[0] = cos(Vehicle_Yaw[2]);
  KinematicsMtx[1] = Dtrailer*sin(Vehicle_Yaw[2]);
  KinematicsMtx[2] = sin(Vehicle_Yaw[2])/Dcar;
  KinematicsMtx[3] = -cos(Vehicle_Yaw[2]);

  CarVelMtx[0] = KinematicsMtx[0]*TrailerVelMtx[0]+KinematicsMtx[1]*TrailerVelMtx[1];
  CarVelMtx[1] = KinematicsMtx[2]*TrailerVelMtx[0]+KinematicsMtx[3]*TrailerVelMtx[1];

  gazebo_angle = atan2f64(CarVelMtx[1]*L,-0.4);
  if(gazebo_angle > 1.7) gazebo_angle-=3.1415;
  else if(gazebo_angle < -1.7) gazebo_angle+=3.1415;
  CarVelo.linear.x = -0.4;
  CarVelo.angular.z = - gazebo_angle / 0.596;

}
int main(int argc, char **argv)
{
  srand((unsigned int)time(NULL));

  ros::init(argc, argv, "pos_error_pub");
  ros::NodeHandle nh;

  ros::Publisher TrailerAngPub = nh.advertise<pose_msgs::msgPose>("trailer_yaw", 100);
  ros::Publisher CarVelopub = nh.advertise<geometry_msgs::Twist>("/des_vel/cmd_vel", 100);
  // Simulation input : /cmd_vel
  // Steering input : /des_vel/cmd_vel
  ros::Subscriber Positionsub = nh.subscribe("/gazebo/link_states", 100, PosErrorCallBack);
  ros::Subscriber Kinematicsub = nh.subscribe("/teleop/cmd_vel",100,VeloKinematicsCallback);
  ros::Rate rate(100); //100hz
  while (ros::ok())
  {
    CarVelo.angular.z = LPF(CarVelo.angular.z, 0.3);
    CarVelopub.publish(CarVelo);
    TrailerAngPub.publish(TrailerAng);
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
