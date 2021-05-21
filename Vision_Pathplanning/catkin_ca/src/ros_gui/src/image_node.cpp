#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>
#include <algorithm>
#include <string>

/*------------Constant------------*/
#define PI              3.141592

#define I2W             1                   // image-->world
#define W2I             0                   // world-->image

#define deg2rad         PI/180.0
#define rad2deg         180/PI

#define LANEWIDTH       1.2                // 차선 폭[m]
#define TRAILERHALF     0.3                 // 트레일러 폭 절반


// Scalar 색상
const cv::Scalar BLACK(0, 0, 0);
const cv::Scalar RED(0, 0, 255);
const cv::Scalar GREEN(0, 180, 0);
const cv::Scalar BLUE(255, 0, 0);
const cv::Scalar WHITE(255, 255, 255);
const cv::Scalar YELLOW(0, 255, 255);
const cv::Scalar CYAN(255, 255, 0);
const cv::Scalar MAGENTA(255, 0, 255);
const cv::Scalar GRAY(150, 150, 150);
const cv::Scalar PURPLE(255, 0, 127);


/*------------Global Variable------------*/
cv::Mat img;
cv::Mat img_bgr;
cv_bridge::CvImagePtr cv_ptr;
sensor_msgs::ImagePtr qmsg;
bool rcvflag = false;

// Draw Line
static double linear_vel = 0.25, angular_vel = 0.0;
static int draw_t = 10;
static int draw_t2 = 6;


/*------------Fuction Prototype------------*/
void TrailerCallBack(const geometry_msgs::Twist::ConstPtr &vel);
void ImageCallback(const sensor_msgs::Image::ConstPtr &img);            // 이미지 subscribe 될 때 호출되는 함수
cv::Point2f transformPoint(const cv::Point2f& cur, const cv::Mat& T);   // 좌표계 변환 함수
void Projection(const std::vector<cv::Point2f>& src, std::vector<cv::Point2f>& dst,
                bool direction = I2W);  // 픽셀좌표계-->월드좌표계 , 월드좌표계-->픽셀좌표계 변환하는 함수 (input vector, output vector, 변환 방법)
void drawPath(const cv::Mat& img_draw);

/*------------Fuction Expression------------*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_processing");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_img = it.subscribe("/image_raw", 1, ImageCallback);
  image_transport::Publisher qimage_pub = it.advertise("/image_raw/streaming", 1);

  ros::Subscriber trailer_cmd_sub = nh.subscribe("/teleop/cmd_vel", 1, TrailerCallBack);

  ros::Rate rate(1000); //100hz
  while (ros::ok())
  {
    if(rcvflag)   qimage_pub.publish(qmsg);
    rate.sleep();
    ros::spinOnce();
  }
}

void TrailerCallBack(const geometry_msgs::Twist::ConstPtr &vel){
      linear_vel = vel->linear.x;   // v
      angular_vel = vel->angular.z; // w
}

void ImageCallback(const sensor_msgs::Image::ConstPtr &img){
  try {
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    img_bgr = cv_ptr->image;
    drawPath(img_bgr);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Error to convert!");
    return;
  };
}

cv::Point2f transformPoint(const cv::Point2f& cur, const cv::Mat& T){
  cv::Point2f tPoint;

  tPoint.x = cur.x * T.at<double>(0,0)
      + cur.y * T.at<double>(0,1)
      + T.at<double>(0,2);
  tPoint.y = cur.x * T.at<double>(1,0)
      + cur.y * T.at<double>(1,1)
      + T.at<double>(1,2);
  float z = cur.x * T.at<double>(2,0)
      + cur.y * T.at<double>(2,1)
      + T.at<double>(2,2);

  tPoint.x /= z;
  tPoint.y /= z;

  return tPoint;
}

void Projection(const std::vector<cv::Point2f>& src, std::vector<cv::Point2f>& dst, bool direction){
  std::vector<cv::Point2f> imagePoints;
  std::vector<cv::Point2f> objectPoints;

  imagePoints.push_back(cv::Point2f(33,121));
  imagePoints.push_back(cv::Point2f(148,123));
  imagePoints.push_back(cv::Point2f(131,71));
  imagePoints.push_back(cv::Point2f(51,69));

  objectPoints.push_back(cv::Point2f(-0.3,0.75));
  objectPoints.push_back(cv::Point2f( 0.3,0.75));
  objectPoints.push_back(cv::Point2f( 0.3,1.35));
  objectPoints.push_back(cv::Point2f(-0.3,1.35));

  cv::Mat img2World = cv::getPerspectiveTransform(imagePoints, objectPoints);
  cv::Mat world2Image = img2World.inv();

  for(int i=0; i<src.size(); i++){
    cv::Point2f p;
    if(direction)   p = transformPoint(src[i], img2World);
    else            p = transformPoint(src[i], world2Image);
    dst.push_back(p);
    // std::cout << "Real [" << i << "] (" << p.x << ", " << p.y << std::endl;
  }
}

void drawPath(const cv::Mat& img_draw){
  // cv::createTrackbar("v", "draw path", &linear_vel, 50);
  // cv::createTrackbar("w", "draw path", &angular_vel, 100);
  // cv::createTrackbar("t", "draw path", &draw_t, 100);
  // cv::createTrackbar("t2", "draw path", &draw_t2, 100);
  double v = 0.15, w = angular_vel/5.;
//  v = linear_vel/100.;
//  w = (angular_vel-50)/500.;
  if(img_draw.empty())  return;
  img = img_draw.clone();
  std::vector<cv::Point2f> path, left, right;
  std::vector<cv::Point2f> path2, left2, right2;
  float twoalpha = w*draw_t;
  float twoalpha2 = w*draw_t2;
  float radius;
  if(w==0) radius = -1;
  else radius = fabs(v/w);
  std::cout << "twoalpha:" << twoalpha << std::endl;
  std::cout << "radius:" << radius*rad2deg << std::endl;
  // std::cout << "ld:" << ld << std::endl;
  // std::cout << "radius:" << radius << std::endl;
  if(w > 0){
    for(int i=0; i < twoalpha*rad2deg*100.; i++){
      float theta = (i*deg2rad/100.);
      float drawx = (radius*cos(theta))-radius;
      float drawy = radius*sin(theta);
      float Rx = ((radius+TRAILERHALF)*cos(theta))-radius;
      float Ry = (radius+TRAILERHALF)*sin(theta);
      float Lx = ((radius-TRAILERHALF)*cos(theta))-radius;
      float Ly = (radius-TRAILERHALF)*sin(theta);
      // std::cout << "Rx:" << Rx << ", Ry:" << Ry << std::endl;
      cv::Point2f drawP = {drawx,drawy};
      path.push_back(drawP);
      left.push_back({Lx, Ly});
      right.push_back({Rx, Ry});
    }
  }
  else if(w < 0){
    for(int i=0; i < fabs(twoalpha)*rad2deg*100.; i++){
      float theta = PI - (i*deg2rad/100.);
      float drawx = (radius*cos(theta))+radius;
      float drawy = radius*sin(theta);
      float Lx = ((radius+TRAILERHALF)*cos(theta))+radius;
      float Ly = (radius+TRAILERHALF)*sin(theta);
      float Rx = ((radius-TRAILERHALF)*cos(theta))+radius;
      float Ry = (radius-TRAILERHALF)*sin(theta);
      // std::cout << "x:" << drawx << ", y:" << drawy << std::endl;
      cv::Point2f drawP = {drawx,drawy};
      path.push_back(drawP);
      left.push_back({Lx, Ly});
      right.push_back({Rx, Ry});
    }
  }
  else{// w=0일때
    float drawx = 0;
    float drawy = v*draw_t;
    float Lx = -TRAILERHALF;
    float Ly = v*draw_t;
    float Rx = TRAILERHALF;
    float Ry = v*draw_t;
    // std::cout << "v*draw_t" << v*draw_t << std::endl;
    // std::cout << "x:" << drawx << ", y:" << drawy << std::endl;
    cv::Point2f drawP = {drawx,drawy};
    //path.push_back({0,0});
    left.push_back({Lx, 0.3});
    right.push_back({Rx, 0.3});
    //path.push_back(drawP);
    left.push_back({Lx, Ly});
    right.push_back({Rx, Ry});

    std::vector<cv::Point2f> drawpathP, leftdrawP, rightdrawP;
    //Projection(path, drawpathP, W2I);
    Projection(left, leftdrawP, W2I);
    Projection(right, rightdrawP, W2I);
    //cv::line(img, drawpathP[0], drawpathP[1], BLUE, 2);
    cv::line(img, leftdrawP[0], leftdrawP[1], YELLOW, 2);
    cv::line(img, rightdrawP[0], rightdrawP[1], YELLOW, 2);
  }

  if(w > 0){
    for(int i=0; i < twoalpha2*rad2deg*100.; i++){
      float theta = (i*deg2rad/100.);
      float drawx = (radius*cos(theta))-radius;
      float drawy = radius*sin(theta);
      float Rx = ((radius+TRAILERHALF)*cos(theta))-radius;
      float Ry = (radius+TRAILERHALF)*sin(theta);
      float Lx = ((radius-TRAILERHALF)*cos(theta))-radius;
      float Ly = (radius-TRAILERHALF)*sin(theta);
      // std::cout << "x:" << drawx << ", y:" << drawy << std::endl;
      cv::Point2f drawP = {drawx,drawy};
      path2.push_back(drawP);
      left2.push_back({Lx, Ly});
      right2.push_back({Rx, Ry});
    }
  }
  else if(w < 0){
    for(int i=0; i < fabs(twoalpha2)*rad2deg*100.; i++){
      float theta = PI - (i*deg2rad/100.);
      float drawx = (radius*cos(theta))+radius;
      float drawy = radius*sin(theta);
      float Lx = ((radius+TRAILERHALF)*cos(theta))+radius;
      float Ly = (radius+TRAILERHALF)*sin(theta);
      float Rx = ((radius-TRAILERHALF)*cos(theta))+radius;
      float Ry = (radius-TRAILERHALF)*sin(theta);
      // std::cout << "x:" << drawx << ", y:" << drawy << std::endl;
      cv::Point2f drawP = {drawx,drawy};
      path2.push_back(drawP);
      left2.push_back({Lx, Ly});
      right2.push_back({Rx, Ry});
    }
  }
  else{// w=0일때
    float drawx = 0;
    float drawy = v*draw_t2;
    float Lx = -TRAILERHALF;
    float Ly = v*draw_t2;
    float Rx = TRAILERHALF;
    float Ry = v*draw_t2;
    // std::cout << "v*draw_t2" << v*draw_t2 << std::endl;
    // std::cout << "x:" << drawx << ", y:" << drawy << std::endl;
    cv::Point2f drawP = {drawx,drawy};
    // path2.push_back({0,0.3});
    left2.push_back({Lx, 0.3});
    right2.push_back({Rx, 0.3});
    // path2.push_back(drawP);
    left2.push_back({Lx, Ly});
    right2.push_back({Rx, Ry});

    std::vector<cv::Point2f> drawpathP, leftdrawP, rightdrawP;
    //Projection(path, drawpathP, W2I);
    Projection(left2, leftdrawP, W2I);
    Projection(right2, rightdrawP, W2I);
    //cv::line(img, drawpathP[0], drawpathP[1], BLUE, 2);
    cv::line(img, leftdrawP[0], leftdrawP[1], RED, 2);
    cv::line(img, rightdrawP[0], rightdrawP[1], RED, 2);
  }
  std::vector<cv::Point2f> drawpathP, leftdrawP, rightdrawP;
  cv::Point2f topC, topL, topR;
  cv::Point2f bottomC, bottomL, bottomR;
  int bC, bL, bR;
  Projection(path, drawpathP, W2I);
  Projection(left, leftdrawP, W2I);
  Projection(right, rightdrawP, W2I);

  std::vector<cv::Point2f> drawpathP2, leftdrawP2, rightdrawP2;
  cv::Point2f topC2, topL2, topR2;
  cv::Point2f bottomC2, bottomL2, bottomR2;
  int bC2, bL2, bR2;
  Projection(path2, drawpathP2, W2I);
  Projection(left2, leftdrawP2, W2I);
  Projection(right2, rightdrawP2, W2I);

  cv::Point2f old_p, old_p2;
  bool flagC = 0, flagL = 0, flagR = 0;
  bool flagC2 = 0, flagL2 = 0, flagR2 = 0;

  for(int i=0; i<drawpathP.size(); i++){
//    if((drawpathP[i].x<img.cols)&&(drawpathP[i].x>0)&&(drawpathP[i].y<img.rows)&&(drawpathP[i].y>0)){
      if(flagC == 0){
        old_p = {drawpathP[i].x, drawpathP[i].y};
        topC = old_p;
        bottomC = old_p;
        bC = i;
        flagC = 1;
      }
      cv::Point2f p = {drawpathP[i].x, drawpathP[i].y};
      // cv::line(img, p, old_p, RED, 2);
      old_p = p;
      if(topC.y > p.y) topC = p;
      if(bottomC.y < p.y){
        bottomC = p;
        bC = i;
      }
      // std::cout << "pix x:" << p.x << ", y:" << p.y << std::endl;
      // cv::circle(img, p, 2, MAGENTA, -1);
//    }
  }

  for(int i=0; i<leftdrawP.size(); i++){
//    if((leftdrawP[i].x<img.cols)&&(leftdrawP[i].x>0)&&(leftdrawP[i].y<img.rows)&&(leftdrawP[i].y>0)){
      if(flagL == 0){
        old_p = {leftdrawP[i].x, leftdrawP[i].y};
        topL = old_p;
        flagL = 1;
      }
      cv::Point2f p = {leftdrawP[i].x, leftdrawP[i].y};
      cv::line(img, p, old_p, YELLOW, 2);
      old_p = p;
      if(topL.y > p.y) topL = p;
      if(bottomL.y < p.y){
        bottomL = p;
        bL = i;
      }
      // cv::circle(img, p, 2, CYAN, -1);
//    }
  }

  for(int i=0; i<rightdrawP.size(); i++){
//    if((rightdrawP[i].x<img.cols)&&(rightdrawP[i].x>0)&&(rightdrawP[i].y<img.rows)&&(rightdrawP[i].y>0)){
      if(flagR == 0){
        old_p = {rightdrawP[i].x, rightdrawP[i].y};
        topR = old_p;
        flagR = 1;
      }
      cv::Point2f p = {rightdrawP[i].x, rightdrawP[i].y};
      cv::line(img, p, old_p, YELLOW, 2);
      old_p = p;
      if(topR.y > p.y) topR = p;
      if(bottomR.y < p.y){
        bottomR = p;
        bR = i;
      }
      // cv::circle(img, p, 2, CYAN, -1);
//    }
  }
  if((leftdrawP.size()!=0) && (rightdrawP.size()!=0)){
    if((topL.y != 0) && (topR.y != 0)){
      cv::line(img, topL, topR, YELLOW, 2);
    }
    if((topL.y != 0) && (topR.y == 0)){
      float a = (topC.y - topL.y)/(float)(topC.x - topL.x);
      float b = topC.y - a*topC.x;
      float x = (float)img.cols;
      float y = a*x + b;
      cv::line(img, topL, cv::Point2f{x,y}, YELLOW, 2);
    }
    if((topL.y == 0) && (topR.y != 0)){
      float a = (topR.y - topC.y)/(float)(topR.x - topC.x);
      float b = topC.y - a*topC.x;
      float x = 0;
      float y = a*x + b;
      cv::line(img, topR, cv::Point2f{x,y}, YELLOW, 2);
    }
  }

  for(int i=0; i<drawpathP2.size(); i++){
//    if((drawpathP2[i].x<img.cols)&&(drawpathP2[i].x>0)&&(drawpathP2[i].y<img.rows)&&(drawpathP2[i].y>0)){
      if(flagC2 == 0){
        old_p2 = {drawpathP2[i].x, drawpathP2[i].y};
        topC2 = old_p2;
        bottomC2 = old_p2;
        bC2 = i;
        flagC2 = 1;
      }
      cv::Point2f p = {drawpathP2[i].x, drawpathP2[i].y};
      // cv::line(img, p, old_p, RED, 2);
      old_p2 = p;
      if(topC2.y > p.y) topC2 = p;
      if(bottomC2.y < p.y){
        bottomC2 = p;
        bC2 = i;
      }
      // std::cout << "pix x:" << p.x << ", y:" << p.y << std::endl;
      // cv::circle(img, p, 2, MAGENTA, -1);
//    }
  }

  for(int i=0; i<leftdrawP2.size(); i++){
//    if((leftdrawP2[i].x<img.cols)&&(leftdrawP2[i].x>0)&&(leftdrawP2[i].y<img.rows)&&(leftdrawP2[i].y>0)){
      if(flagL2 == 0){
        old_p2 = {leftdrawP2[i].x, leftdrawP2[i].y};
        topL2 = old_p2;
        flagL2 = 1;
      }
      cv::Point2f p = {leftdrawP2[i].x, leftdrawP2[i].y};
      cv::line(img, p, old_p2, RED, 2);
      old_p2 = p;
      if(topL2.y > p.y) topL2 = p;
      if(bottomL2.y < p.y){
        bottomL2 = p;
        bL2 = i;
      }
      // cv::circle(img, p, 2, CYAN, -1);
//    }
  }

  for(int i=0; i<rightdrawP2.size(); i++){
//    if((rightdrawP2[i].x<img.cols)&&(rightdrawP2[i].x>0)&&(rightdrawP2[i].y<img.rows)&&(rightdrawP2[i].y>0)){
      if(flagR2 == 0){
        old_p2 = {rightdrawP2[i].x, rightdrawP2[i].y};
        topR2 = old_p2;
        flagR2 = 1;
      }
      cv::Point2f p = {rightdrawP2[i].x, rightdrawP2[i].y};
      cv::line(img, p, old_p2, RED, 2);
      old_p2 = p;
      if(topR2.y > p.y) topR2= p;
      if(bottomR2.y < p.y){
        bottomR2 = p;
        bR2 = i;
      }
      // cv::circle(img, p, 2, CYAN, -1);
//    }
  }
  if((leftdrawP2.size()!=0) && (rightdrawP2.size()!=0)){
    if((topL2.y != 0) && (topR2.y != 0)){
      cv::line(img, topL2, topR2, RED, 2);
      // std::cout << "topL y:" << topL2.y << ", topR y:" << topR2.y << std::endl;
    }
    if((topL2.y != 0) && (topR2.y == 0)){
      float a = (topC2.y - topL2.y)/(float)(topC2.x - topL2.x);
      float b = topC2.y - a*topC2.x;
      float x = (float)img.cols;
      float y = a*x + b;
      cv::line(img, topL2, cv::Point2f{x,y}, RED, 2);
      // std::cout << "topL y:" << topL2.y << ", topC y:" << y << std::endl;
    }
    if((topL2.y == 0) && (topR2.y != 0)){
      // float a = (topC2.y - topR2.y)/(float)(topC2.x - topR2.x);
      // float b = topC2.y - a*topC2.x;
      float a = (topC2.y - topR2.y)/(float)(topC2.x - topR2.x);
      float b = topC2.y - a*topC2.x;
      float x = 0;
      float y = a*x + b;
      cv::line(img, topR2, cv::Point2f{x,y}, RED, 2);
      // std::cout << "topC y:" << y << ", topR y:" << topR.y << std::endl;
      // std::cout << "!!!!!!!" << std::endl;
    }
  }
  rcvflag = true;
  cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
  qmsg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", img).toImageMsg();
}
