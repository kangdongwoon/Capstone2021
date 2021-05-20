#ifndef ROS_CONNECT_H
#define ROS_CONNECT_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "Joy.h"
#include "stdlib.h"
#include "iostream"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"


#include <QObject>
#include <QPoint>
#include <QDebug>
#include <sstream>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <pose_msgs/msgPose.h>
#include <sensor_msgs/Joy.h>

#include <math.h>
#define min(x, y) (((x) < (y)) ? (x) : (y))
#define max(x, y) (((x) > (y)) ? (x) : (y))

class ros_connect : public QObject
{
  Q_OBJECT
public:
  explicit ros_connect(QObject *parent, ros::NodeHandle &nh)
      {
        m_Q = parent;    tt = 0;
        imu_sub = nh.subscribe("/imu", 1, &ros_connect::imu_cb, this);
        trailpose_sub = nh.subscribe("/trailer_pose", 1 , &ros_connect::trailer_pose, this);
        trailyaw_sub = nh.subscribe("/trailer_yaw", 1 , &ros_connect::trailer_yaw, this);
        carvel_sub = nh.subscribe("/joystick/cmd_vel", 1 , &ros_connect::car_vel, this);
        // Simulation input : /cmd_vel
        // Steering input : /joystick/cmd_vel
        joystick_sub = nh.subscribe("/controller/gui_command", 1 , &ros_connect::joystick, this);
      };

      Q_INVOKABLE void imu_cb(const sensor_msgs::Imu::ConstPtr &imu)
      {
        char buf[128];

        std::sprintf(buf, "%8.4f", imu->orientation.z);
        m_Q->findChild<QObject *>("t1")->setProperty("text", buf);
        std::sprintf(buf, "%8.4f", imu->orientation.y);
        m_Q->findChild<QObject *>("t2")->setProperty("text", buf);
        std::sprintf(buf, "%8.4f", imu->orientation.x);
        m_Q->findChild<QObject *>("t3")->setProperty("text", buf);
        std::sprintf(buf, "%8.4f", imu->orientation.w);
        m_Q->findChild<QObject *>("t4")->setProperty("text", buf);
        std::sprintf(buf, "%8.4f", imu->angular_velocity.x);
        m_Q->findChild<QObject *>("t5")->setProperty("text", buf);
        std::sprintf(buf, "%8.4f", imu->angular_velocity.y);
        m_Q->findChild<QObject *>("t6")->setProperty("text", buf);
        std::sprintf(buf, "%8.4f", imu->angular_velocity.z);
        m_Q->findChild<QObject *>("t7")->setProperty("text", buf);
        std::sprintf(buf, "%8.4f", imu->linear_acceleration.x);
        m_Q->findChild<QObject *>("t8")->setProperty("text", buf);
        std::sprintf(buf, "%8.4f", imu->linear_acceleration.y);
        m_Q->findChild<QObject *>("t9")->setProperty("text", buf);
        std::sprintf(buf, "%8.4f", imu->linear_acceleration.z);
        m_Q->findChild<QObject *>("t10")->setProperty("text", buf);

        std::sprintf(buf, "%8.4f", imu->orientation.z);
        m_Q->findChild<QObject *>("p1")->setProperty("value", buf);
        std::sprintf(buf, "%8.4f", imu->orientation.y);
        m_Q->findChild<QObject *>("p2")->setProperty("value", buf);
        std::sprintf(buf, "%8.4f", imu->orientation.x);
        m_Q->findChild<QObject *>("p3")->setProperty("value", buf);
        std::sprintf(buf, "%8.4f", imu->orientation.w);
        m_Q->findChild<QObject *>("p4")->setProperty("value", buf);
        std::sprintf(buf, "%8.4f", imu->angular_velocity.x);
        m_Q->findChild<QObject *>("p5")->setProperty("value", buf);
        std::sprintf(buf, "%8.4f", imu->angular_velocity.y);
        m_Q->findChild<QObject *>("p6")->setProperty("value", buf);
        std::sprintf(buf, "%8.4f", imu->angular_velocity.z);
        m_Q->findChild<QObject *>("p7")->setProperty("value", buf);
        std::sprintf(buf, "%8.4f", imu->linear_acceleration.x);
        m_Q->findChild<QObject *>("p8")->setProperty("value", buf);
        std::sprintf(buf, "%8.4f", imu->linear_acceleration.y);
        m_Q->findChild<QObject *>("p9")->setProperty("value", buf);
        std::sprintf(buf, "%8.4f", imu->linear_acceleration.z);
        m_Q->findChild<QObject *>("p10")->setProperty("value", buf);
      }

      Q_INVOKABLE void trailer_pose(const pose_msgs::msgPose::ConstPtr &pose){
        char buf[128];
        std::sprintf(buf, "%8.4f", pose->pose_x);
        m_Q->findChild<QObject *>("trailer_xvalue")->setProperty("text", buf);
        std::sprintf(buf, "%8.4f", pose->pose_y);
        m_Q->findChild<QObject *>("trailer_yvalue")->setProperty("text", buf);
        std::sprintf(buf, "%8.4f", pose->car_x);
        m_Q->findChild<QObject *>("car_xvalue")->setProperty("text", buf);
        std::sprintf(buf, "%8.4f", pose->car_y);
        m_Q->findChild<QObject *>("car_yvalue")->setProperty("text", buf);
      }
      Q_INVOKABLE void trailer_yaw(const pose_msgs::msgPose::ConstPtr &yaw){
        char buf[128];
        std::sprintf(buf, "%8.4f", yaw->trailer_yaw);
        m_Q->findChild<QObject *>("btwangle")->setProperty("text", buf);
      }
      Q_INVOKABLE void car_vel(const geometry_msgs::Twist::ConstPtr &vel){
        char buf[128];
        std::sprintf(buf, "%8.4f", vel->linear.x);
        m_Q->findChild<QObject *>("linear_value")->setProperty("text", buf);
        m_Q->findChild<QObject *>("lineargaze")->setProperty("value", buf);
        std::sprintf(buf, "%8.4f", vel->angular.z);
        m_Q->findChild<QObject *>("steer_value")->setProperty("text", buf);
        m_Q->findChild<QObject *>("steergaze")->setProperty("value", buf);
        std::sprintf(buf, "%8.4f", (vel->angular.z)*225.0);
        m_Q->findChild<QObject *>("tar_steeringdial")->setProperty("value", buf);

      }
      Q_INVOKABLE void joystick(const sensor_msgs::Joy::ConstPtr &joy)
      {
        char buf[128];
        std::sprintf(buf, "%8.4f",  ( (-1.0)* (double)joy->axes[0] * 0.4)*225.0);
        m_Q->findChild<QObject *>("cur_steeringdial")->setProperty("value", buf);
      }

      Q_INVOKABLE void update()
      {
        ros::spinOnce();
      };
      Q_INVOKABLE double t_x(){
        return tt;
      };


      Q_INVOKABLE int tt;
      Q_INVOKABLE QString a;


  ///////////////////////////////////////////////////////////////////////////////////////////////

  ros::Subscriber imu_sub;
  ros::Subscriber trailpose_sub;
  ros::Subscriber trailyaw_sub;
  ros::Subscriber carvel_sub;
  ros::Subscriber joystick_sub;

  float pp(float val){
    if (val < 0){
      return (val * -1.0) / 150.0;
    }
    else{
      return val / 150.0;
    }
  };

protected:
  QObject *m_Q;

signals:

public slots:
};

#endif // ROS_CONNECT_H
