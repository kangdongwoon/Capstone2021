#ifndef ROS_CONNECT_H
#define ROS_CONNECT_H

#include "ros/ros.h"
#include "stdlib.h"
#include "iostream"
#include <QFont>
#include <QPoint>
#include <QDebug>
#include <QObject>
#include <pose_msgs/msgPose.h>
#include "wrp_sdk/platforms/hunter/hunter_base.hpp"
#include <hunter_msgs/HunterStatus.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PolygonStamped.h>
#include <math.h>

#define PI  3.1415926535
#define min(x, y) (((x) < (y)) ? (x) : (y))
#define max(x, y) (((x) > (y)) ? (x) : (y))
static bool reverse;
static int start = 0;

class ros_connect : public QObject
{
  Q_OBJECT
public:
  explicit ros_connect(QObject *parent, ros::NodeHandle &nh){
    m_Q = parent;
    init_pub = nh.advertise<std_msgs::String>("/initialize", 100);

    desiredwheel_sub = nh.subscribe("/des_vel/cmd_vel", 1 , &ros_connect::desired_wheel, this);
    racingwheel_sub = nh.subscribe("/controller/gui_command", 1, &ros_connect::racing_wheel, this);
    minitrailer_sub = nh.subscribe("/trailer_pose", 1, &ros_connect::mini_trailer, this);
    hunter_sub = nh.subscribe("/hunter_status", 1, &ros_connect::hunter_status, this);
  }
    Q_INVOKABLE void mini_trailer(const pose_msgs::msgPose &yaw){
        char buf[128];
        std::sprintf(buf, "%8.4f", -yaw.encoder_theta);
        m_Q->findChild<QObject *>("minitrailer_yaw")->setProperty("rotation", buf);
    }
    Q_INVOKABLE void hunter_status(const hunter_msgs::HunterStatus &hstatus){
        char buf[128];
        std::sprintf(buf, "%8.4f", (double)hstatus.steering_angle * -200. / PI);
        m_Q->findChild<QObject *>("minitrailer_leftwheel")->setProperty("rotation", buf);
        m_Q->findChild<QObject *>("minitrailer_rightwheel")->setProperty("rotation", buf);
    }

  Q_INVOKABLE void desired_wheel(const geometry_msgs::Twist::ConstPtr &dwheel){
    char buf[128];
    if((double)dwheel->angular.z > 0.576){
        std::sprintf(buf, "%8.4f", -135.);
    }
    else if((double)dwheel->angular.z < -0.576){
        std::sprintf(buf, "%8.4f", 135.);
    }
    else{
        std::sprintf(buf, "%8.4f", ((double)dwheel->angular.z / 0.576) * -135.);
    }
    m_Q->findChild<QObject *>("desiredwheel")->setProperty("rotation", buf);
  }

  Q_INVOKABLE void racing_wheel(const sensor_msgs::Joy::ConstPtr &rwheel){
    char buf[128];
    std::sprintf(buf, "%8.4f", ((double)rwheel->axes[0] * -135.0));
    m_Q->findChild<QObject *>("racingwheel")->setProperty("rotation", buf);
    m_Q->findChild<QObject *>("racingwheel_value")->setProperty("currentValue", buf);

    if(rwheel->buttons[7] == 1){
        std_msgs::String init_cmd;
        init_cmd.data = "initialize";
        init_pub.publish(init_cmd);
    }

    if(rwheel->axes[2] == -1 && rwheel->buttons[3] == 1.){
         reverse = !reverse;
    }
    if(rwheel->buttons[0] == 1){
         start = 1;
    }
    if(start){
          if(reverse){
               m_Q->findChild<QObject *>("letterR")->setProperty("color", "white");
               f.setPixelSize(36);
               m_Q->findChild<QObject *>("letterR")->setProperty("font", f);
               m_Q->findChild<QObject *>("letterD")->setProperty("color", "darkgray");
               f.setPixelSize(18);
               m_Q->findChild<QObject *>("letterD")->setProperty("font", f);
               m_Q->findChild<QObject *>("letterP")->setProperty("color", "darkgray");
               f.setPixelSize(18);
               m_Q->findChild<QObject *>("letterP")->setProperty("font", f);
          }
          else{
               m_Q->findChild<QObject *>("letterD")->setProperty("color", "white");
               f.setPixelSize(36);
               m_Q->findChild<QObject *>("letterD")->setProperty("font", f);
               m_Q->findChild<QObject *>("letterP")->setProperty("color", "darkgray");
               f.setPixelSize(18);
               m_Q->findChild<QObject *>("letterP")->setProperty("font", f);
               m_Q->findChild<QObject *>("letterR")->setProperty("color", "darkgray");
               f.setPixelSize(18);
               m_Q->findChild<QObject *>("letterR")->setProperty("font", f);
          }
      }
  }

  Q_INVOKABLE void update(){
    ros::spinOnce();
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////

  ros::Publisher init_pub;

  ros::Subscriber desiredwheel_sub;
  ros::Subscriber racingwheel_sub;
  ros::Subscriber minitrailer_sub;
  ros::Subscriber hunter_sub;

protected:
  QObject *m_Q;
  QFont f;
signals:

public slots:

};

#endif // ROS_CONNECT_H
