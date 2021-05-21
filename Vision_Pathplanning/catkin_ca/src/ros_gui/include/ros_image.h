#ifndef ROS_IMAGE_H
#define ROS_IMAGE_H

#include <string>
#include <QImage>
#include <QObject>
#include <QQuickItem>
#include <QQuickImageProvider>
#include <opencv2/opencv.hpp>

#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

class ros_image : public QObject, public QQuickImageProvider
{
  Q_OBJECT
public:
  ros_image(ros::NodeHandle &nh, std::string image_topic): QQuickImageProvider(QQuickImageProvider::Pixmap),
    it(ros::NodeHandle()), qimg(QImage(10,10, QImage::Format_RGB888)){
    image_sub = it.subscribe(image_topic, 1, &ros_image::image_streaming, this);
    qimg.fill(QColor("black").rgba());
  }

  QImage requestImage(const QString &id, QSize *size, const QSize &requestedSize){
    QImage result;
    if (requestedSize.isValid())
      result = qimg.scaled(requestedSize, Qt::KeepAspectRatio);
    else    result = qimg;

    *size = result.size();
    return result;
  }

  ImageType imageType() const override {return QQmlImageProviderBase::Image;}

  void image_streaming(const sensor_msgs::ImageConstPtr& msg){
    qimg = QImage(msg->width, msg->height, QImage::Format_RGB888);
    memcpy(qimg.bits(), msg->data.data(), qimg.byteCount());
  }


private:
  std::string image_topic;

  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;

  QImage qimg;
  cv::Mat img_bgr;
};



#endif // ROS_IMAGE_H
