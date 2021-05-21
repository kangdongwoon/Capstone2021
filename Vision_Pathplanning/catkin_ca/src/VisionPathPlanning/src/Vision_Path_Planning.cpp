#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "iostream"
#include "pose_msgs/msgPose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "math.h"
#include "wrp_sdk/platforms/hunter/hunter_base.hpp"
#include "hunter_msgs/HunterStatus.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_broadcaster.h"
#include "std_msgs/Int16MultiArray.h"

#define SQR(x) ((x)*(x))
#define PI  3.1415926535
#define RESOLUTION 100
using namespace std;

static geometry_msgs::Pose CarPose;
static geometry_msgs::Pose TrailerPose;
static geometry_msgs::Twist TrailerVelo;
static pose_msgs::msgPose tpose;
static std_msgs::Float64 enco;
static hunter_msgs::HunterStatus Hstatus;

static double lookx;
static double looky;
static double Trailer_Yaw=0;
static double Car_Yaw=0;

static double dl = 1.5;
static double alpha = 0.;

static double Trailer_vel = 0.;
static double Trailer_ang = 0.;
static double Trailer_ang_PD = 0.;

static double dtrailer = 0.745;
static double dcar = 0.30;

static double car_angular_vel = 0;
static double car_linear_vel = 0;

static double L = 0.648;

static tf::Quaternion quarterion;
static tf::Transform cartf;
static tf::Transform trailertf;
static tf::Quaternion q;
static tf::Transform Tr;
static tf::Quaternion torient;

void EncoderCallBack(const std_msgs::Int16::ConstPtr& enc){
  enco.data = -(double)enc->data/1000.0;
  tpose.encoder_theta = enco.data*180/PI;
}


void CarVelCallback(const geometry_msgs::Twist::ConstPtr& vel){                 //차량의 속도

  car_linear_vel = vel->linear.x;
  // Linear Mapping "cmd_vel" -> Tire Steering Angle
  car_angular_vel = (0.596)*vel->angular.z;
  // Tire Steering Angle -> "Car_Angular_vel"
  car_angular_vel = ((-car_linear_vel)/L)*tanf64(car_angular_vel);

}
void HunterstatusCallback(const hunter_msgs::HunterStatus::ConstPtr& status){     //차량의 상태
    Hstatus.steering_angle = status->steering_angle;
    Hstatus.linear_velocity = status->linear_velocity;
}

void LookaheadpointCallback(const std_msgs::Int16MultiArray::ConstPtr& look)    //VISION LOOKAHEADPOINT
{
  looky = (look->data[0])/100.;//y축 m
  lookx = -(look->data[1])/100.;//x축 m
}

void HunterPosErrorCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& odm)  //pose estimation + pathplanning
{
/* Recv odom data from "robot_pose_ekf" - robot_pose_ekf.launch
 * TOPIC NAME : "/robot_pose_ekf/odom_combined"
 * Launch File을 키는 시점으로 Odom Data 측정 -> World 좌표에서부터 차량의 Yaw값을 추정함
*/
  CarPose.orientation = odm->pose.pose.orientation;
  Car_Yaw = (double)(atan2f64(2.0*(CarPose.orientation.w*CarPose.orientation.z - CarPose.orientation.x*CarPose.orientation.y),
                            -SQR(CarPose.orientation.x)+SQR(CarPose.orientation.y)-SQR(CarPose.orientation.z)+SQR(CarPose.orientation.w)));
  tpose.car_x = CarPose.position.x = odm->pose.pose.position.x;
  tpose.car_y = CarPose.position.y = odm->pose.pose.position.y;
                CarPose.position.z = odm->pose.pose.position.z;
  tpose.car_yaw = Car_Yaw;
/* 차량의 TF -> 트레일러의 TF 변환
*/
  cartf.setOrigin(tf::Vector3(CarPose.position.x,CarPose.position.y,CarPose.position.z));
  cartf.setRotation(tf::Quaternion(CarPose.orientation.x,CarPose.orientation.y,CarPose.orientation.z,CarPose.orientation.w));
  Tr.setOrigin(tf::Vector3(-dtrailer*cos(enco.data)-dcar, -dtrailer*sin(enco.data) ,0.0));
  Tr.setRotation(tf::Quaternion(0,0,sin(enco.data)/2.0,cos(enco.data)/2.0));
  trailertf = cartf*Tr;

/* Plot 트레일러 & Joint Position
*/
  tpose.pose_x = trailertf.getOrigin().getX();
  tpose.pose_y = trailertf.getOrigin().getY();
  tpose.link_x = tpose.car_x - dcar*cos(Car_Yaw);
  tpose.link_y = tpose.car_y - dcar*sin(Car_Yaw);
  torient = trailertf.getRotation();

/* Car Yaw에서 Trailer Yaw값 추정 in Radian
*/
  Trailer_Yaw = enco.data + Car_Yaw;
  tpose.trailer_yaw = Trailer_Yaw;


  // Vision Path //
  alpha = atan2(looky,-lookx);
  alpha = -alpha; // LookAhead Point가 트레일러 기준 왼쪽에 위치할 때 '양수' : Trailer의 각속도 w 가 양수여야하므로

  // Vision에서 변화하는 dl
  dl = sqrt(SQR(looky)+SQR(lookx));
  // alpha 값 확인용
  tpose.pose_static = alpha;
  // Lookahead Point 확인용
  tpose.look_x = dl*cos(PI+Trailer_Yaw+alpha) + tpose.pose_x;
  tpose.look_y = dl*sin(PI+Trailer_Yaw+alpha) + tpose.pose_y;

  // car_linear_vel 에서 나오는 차량의 속도가 맞는지 확인해보기
  // car_angular_vel 에서 나오는 차량의 w가 맞나?
  Trailer_vel = cos(enco.data)*car_linear_vel + dcar*sin(enco.data)*car_angular_vel;
  Trailer_ang = 2.*(sin(alpha)/dl)*abs(Trailer_vel);
  tpose.trailer_vel = Trailer_vel;
  tpose.trailer_angular = -Trailer_ang;
  Trailer_ang_PD = Trailer_ang;// + (kp*error + kd*(pre_err - error)/dt);

  TrailerVelo.linear.x = Trailer_vel;
  TrailerVelo.angular.z = -Trailer_ang;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Vision_Path_Planning");
  ros::NodeHandle nh;

  ros::Publisher Trailerposeopub = nh.advertise<pose_msgs::msgPose>("/trailer_pose", 100);
  ros::Publisher TrailerVelopub = nh.advertise<geometry_msgs::Twist>("/teleop/cmd_vel", 100);
  ros::Subscriber encodersub = nh.subscribe("/encoder_theta", 100, EncoderCallBack);
  ros::Subscriber Positionsub = nh.subscribe("/robot_pose_ekf/odom_combined", 100, HunterPosErrorCallBack);
  ros::Subscriber CarVelosub = nh.subscribe("/cmd_vel",100,CarVelCallback);
  ros::Subscriber Hunterstatussub = nh.subscribe("/hunter_status", 100,HunterstatusCallback);
  ros::Subscriber Lookaheadpointsub = nh.subscribe("/lookahead_Point", 100,LookaheadpointCallback);
  ros::Rate rate(200); //200hz

  while (ros::ok())
  {

    TrailerVelopub.publish(TrailerVelo);
    Trailerposeopub.publish(tpose);
    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
