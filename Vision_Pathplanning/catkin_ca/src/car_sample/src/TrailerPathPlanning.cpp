#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "gazebo_msgs/ModelStates.h"
#include "iostream"
#include "pose_msgs/msgPose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <math.h>
#include "tf/transform_datatypes.h"
#include "tf/transform_broadcaster.h"

#define SQR(x) ((x)*(x))
#define PI  3.1415926535
#define RESOLUTION 60
using namespace std;

static geometry_msgs::Pose CarPose;
static geometry_msgs::Pose TrailerPose;
static geometry_msgs::Twist TrailerVelo;
static pose_msgs::msgPose tpose;

static double Trailer_Yaw=0;
static double Car_Yaw=0;
static double Btw_angle=0;

static struct Point{
  double x;
  double y;
}POINT;

static Point CirclePoints[RESOLUTION+1];
static Point LookAHeadPt;
static double dl = 1;
static double pt_err = 0;
static double alpha = 0;
static double Trailer_vel = 0;
static double Trailer_ang = 0;
static double ka = 1.3;

// Varying Trailer Velocity Variables by CONST Car Linear velocity
static double car_angular_vel = 0;
static double car_linear_vel = 0;
static double Dcar = 0.4216;
static double Dtrailer = 0.4230;

void CarVelCallback(const geometry_msgs::Twist::ConstPtr& vel){
  car_angular_vel = vel->angular.z;
  car_linear_vel = -1*(vel->linear.x);
}

void PosErrorCallBack(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  // Trailer Position N Orientation
  TrailerPose.position.x = (msg->pose[7].position.x + msg->pose[8].position.x)/2;
  TrailerPose.position.y = (msg->pose[7].position.y + msg->pose[8].position.y)/2;
  TrailerPose.orientation = msg->pose[6].orientation;
  tpose.pose_x = (msg->pose[7].position.x + msg->pose[8].position.x)/2.0;
  tpose.pose_y = (msg->pose[7].position.y + msg->pose[8].position.y)/2.0;
  // Car Position N Orientation
  tpose.car_x = msg->pose[1].position.x;
  tpose.car_y = msg->pose[1].position.y;
  CarPose.orientation = msg->pose[1].orientation;
  // Car Yaw Angle from Gazebo WORLD Frame
  Car_Yaw = (double)(atan2f(2.0*(CarPose.orientation.w*CarPose.orientation.z - CarPose.orientation.x*CarPose.orientation.y),
                            -SQR(CarPose.orientation.x)+SQR(CarPose.orientation.y)-SQR(CarPose.orientation.z)+SQR(CarPose.orientation.w)));
  // Trailer Yaw Angle from Gazebo WORLD Frame
  Trailer_Yaw = (double)(atan2f(2.0*(TrailerPose.orientation.w*TrailerPose.orientation.z - TrailerPose.orientation.x*TrailerPose.orientation.y),
      -SQR(TrailerPose.orientation.x)+SQR(TrailerPose.orientation.y)-SQR(TrailerPose.orientation.z)+SQR(TrailerPose.orientation.w)));
  // Car - Trailer BTW Angle
  Btw_angle = Car_Yaw - Trailer_Yaw;
  // dl Evaluation Mechanism //
  // Default Minimum of LD is 5.29
  //dl = (msg->twist[6].linear.x * 3.6) * 0.237 + 5.29;

  // LookAHead Point Algorithm
  pt_err = 1000;
  for(int i=0; i<RESOLUTION; i++){
    CirclePoints[i].x = TrailerPose.position.x + dl*cos(i*2*PI/RESOLUTION);
    CirclePoints[i].y = TrailerPose.position.y + dl*sin(i*2*PI/RESOLUTION);

    if(Trailer_Yaw>0){ // Y<1/tan(-yaw)(X-Xc)+Yc
      if(CirclePoints[i].y<(1/tan(-Trailer_Yaw))*(CirclePoints[i].x-TrailerPose.position.x)+TrailerPose.position.y){
          if(pt_err>abs(CirclePoints[i].y-ka*cos((CirclePoints[i].x)/2.0))){
            pt_err = abs(CirclePoints[i].y-ka*cos((CirclePoints[i].x)/2.0));
            LookAHeadPt.x = CirclePoints[i].x;
            LookAHeadPt.y = ka*cos((CirclePoints[i].x)/2.0);
            tpose.pose_cos = ka*cos((tpose.pose_x)/2.0);
        }
      }
    }
    else{
      if(CirclePoints[i].y>(1/tan(-Trailer_Yaw))*(CirclePoints[i].x-TrailerPose.position.x)+TrailerPose.position.y){
        if(pt_err>abs(CirclePoints[i].y-ka*cos((CirclePoints[i].x)/2.0))){
          pt_err = abs(CirclePoints[i].y-ka*cos((CirclePoints[i].x)/2.0));
          LookAHeadPt.x = CirclePoints[i].x;
          LookAHeadPt.y = ka*cos((CirclePoints[i].x)/2.0);
          tpose.pose_cos = ka*cos((tpose.pose_x)/2.0);
        }
      }
    }
  }

  // Lookahead Point - Trailer Point Alpha
  alpha = atan2(LookAHeadPt.y-TrailerPose.position.y,LookAHeadPt.x-TrailerPose.position.x);
  alpha = (PI-(alpha-Trailer_Yaw));

  // Trailer Velocity dicision by CONST Car Velocity
  Trailer_vel =( cos(Btw_angle)*car_linear_vel + Dtrailer*sin(Btw_angle)*car_angular_vel )/( SQR(cos(Btw_angle))+(Dtrailer/Dcar)*SQR(sin(Btw_angle)) );

  Trailer_ang = 2*(sin(alpha)/dl)*Trailer_vel;
  TrailerVelo.linear.x = Trailer_vel;
  TrailerVelo.angular.z = Trailer_ang;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TrailerPathPlanning");
  ros::NodeHandle nh;
  ros::Publisher Trailerposeopub = nh.advertise<pose_msgs::msgPose>("trailer_pose", 100);
  ros::Publisher TrailerVelopub = nh.advertise<geometry_msgs::Twist>("/teleop/cmd_vel", 100);
  ros::Subscriber Positionsub = nh.subscribe("/gazebo/link_states", 100, PosErrorCallBack);
  ros::Subscriber CarVelosub = nh.subscribe("/cmd_vel",100,CarVelCallback);
  ros::Rate rate(100); //100hz
  while (ros::ok())
  {
    TrailerVelopub.publish(TrailerVelo);
    Trailerposeopub.publish(tpose);

    ros::spinOnce();
  }
  return 0;
}
