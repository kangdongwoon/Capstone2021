#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int16MultiArray.h>
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
#define PI 3.141592

#define MORPH_MASK 3    // 모폴로지 연산 mask 사이즈 n X n
#define GAUSSIAN_MASK 3 // 가우시안 연산 mask 사이즈 n X n
#define MA_FILTER 5     // moving average filter 사이즈

// Chessboard Parameter
#define CHESSBOARDGRID 0.025 // Chessboard 사각형 한 변 길이 [m]
#define CHESS_ROWS 8         // Chessboard 헁 꼭지점 수 (행 사각형 갯수-1)
#define CHESS_COLS 6         // Chessboard 열 꼭지점 수 (열 사각형 갯수 1)

// Camera Intrinsic Parameter
#define FX 786.5830110000001   // focal length_x
#define FY 797.147364          // focal length_y
#define CX 390.533843          // principal point_x
#define CY 322.382708          // principal point_y
#define K1 0.06600499999999999 // radial distortion
#define K2 -0.064593           // radial distortion
#define P1 0.006004            // tangential distortion
#define P2 -0.000721           // tangential distortion

#define I2W 1 // image-->world
#define W2I 0 // world-->image

#define deg2rad PI / 180.0
#define rad2deg 180 / PI

#define LD 69           // Lookahead Distance 1.5[m]일때 y축 픽셀좌표
#define SECTION1 96     // SECTION1 시작 y좌표 (1.0[m]) //348
#define SECTION2 81     // SECTION2 시작 y좌표 (1.3[m]) //286.5
#define SECTION3 69     // SECTION3 시작 y좌표 (1.5[m]) //240
#define SECTIONEND 59   // SECTION3 끄ㅌ y좌표 (1.7[m]) //204
#define LANEWIDTH 1.2   // 차선 폭[m]
#define TRAILERHALF 0.3 // 트레일러 폭 절반

#define HOUGH 0  // HoughLines
#define HOUGHP 1 // HoughLinesP

#define FONTSIZE 6 // 좌표 text 사이즈

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

/*------------Struct------------*/

typedef struct
{
    double X;    // 카메라 X위치 [m]
    double Y;    // 카메라 Y위치 [m]
    double Z;    // 카메라 Z위치 [m]
    double pan;  // 카메라 좌우 회전각 [rad] (왼쪽 +, 오른쪽 -)
    double tilt; // 카메라 상하 회전각 [rad] (위  +,  아래  -)
    //double roll;          // 카메라 광학축 기준 회전각 [rad] (카메라와 같은 방향을 바라볼 때, 시계방향 +, 반시계방향 -)
    cv::Mat Rotation;     // 카메라 외부 파라미터 - Rotation Matrix
    cv::Mat Translation;  // 카메라 외부 파라미터 - Translation Matrix
    cv::Mat cameraMatrix; // 카메라 내부 파라미터 - Focal Length, Principal Point
    cv::Mat distCoeffs;   // 카메라 내부 파라미터 - Distortion Coefficients
} camParam;               // 카메라 파라미터 구조체

/*------------Global Variable------------*/
ros::Publisher read_pub;
std_msgs::Int16MultiArray lookahead_point;
bool pubflag = false;
bool left_lflag = true, right_lflag = true;

camParam camera;
cv::Size patternsize(CHESS_ROWS, CHESS_COLS); //checker board interior number of corners

cv::Mat img_comb;
std::vector<cv::Point> poly, roi1;
cv::Point2f old_s1, old_wp;

// MouseClick
cv::Mat img_click;
std::vector<cv::Point2f> clicked;
static int click_cnt = 1;

std::vector<cv::Point2f> MAF_buf, SEC1_buf, BOTTOM_buf; // Moving Average Filter 담는 벡터

// Save Video Parameter
const double fps = 30.0;                                  // 비디오 프레임수
int fourcc = cv::VideoWriter::fourcc('X', '2', '6', '4'); // 비디오 코덱 'M', 'P', '4', 'V'
// cv::VideoWriter video_line("/home/annie/line_quarter_path3.mp4", fourcc, fps, cv::Size(176, 144), true);   // 비디오 파일명과 사이즈 등
// cv::VideoWriter video_ori("/home/annie/ori_quarter_path3.mp4", fourcc, fps, cv::Size(176, 144), true);   // 비디오 파일명과 사이즈 등
// cv::VideoWriter video_bin("/home/annie/binaary_path3.mp4", fourcc, fps, cv::Size(176, 144), true);   // 비디오 파일명과 사이즈 등
// cv::VideoWriter video_edge("/home/annie/edge_path3.mp4", fourcc, fps, cv::Size(176, 144), true);   // 비디오 파일명과 사이즈 등
// cv::VideoCapture cap("yellow_line_test44.mp4");
// cv::VideoCapture cap("/home/annie/ori_quarter.mp4");

// putText Parameter
static int font = cv::FONT_HERSHEY_SIMPLEX; // normal size sans-serif font
static double fontScale = FONTSIZE / 10.0;
static int thickness = (int)fontScale + 2;
static int baseLine = 0;

// Camera Intrinsic Parameter
double camera_matrix[] = {FX, 0., CX, 0., FY, CY, 0., 0., 1.};
cv::Mat cameraMatrix(3, 3, CV_64FC1, camera_matrix);

// Camera Distortion Coefficients
double distortion_coeffs[] = {K1, K2, P1, P2};
cv::Mat distCoeffs(4, 1, CV_64FC1, distortion_coeffs);

static bool cali_flag = 1; // 첫 실행 확인 flag

// Trackbar Variable(노란색)
static int lowH = 19, highH = 75;
static int lowS = 45, highS = 255;
static int lowV = 100, highV = 255;

// Canny edge Threshold
static int lowTH = 50, highTH = 150;

// Hough Threshold
static int houghTH = 40;
static int houghPTH = 19, minLine = 12, maxGap = 25;

// Canny edge Threshold
static int linear_vel = 25, angular_vel = 50;
static int draw_t = 10;
static int draw_t2 = 5;

/*------------Fuction Prototype------------*/
void ImageCallback(const sensor_msgs::Image::ConstPtr &img);            // 이미지 subscribe 될 때 호출되는 함수
void MouseCallback(int event, int x, int y, int flags, void *userdata); // 창 마우스 클릭될 때 호출되는 함수

void setROI(const cv::Mat &src, cv::Mat &dst, const std::vector<cv::Point> &points);      // fillpoly함수로 다각형 ROI설정 함수
void setROIGray(const cv::Mat &src, cv::Mat &dst, const std::vector<cv::Point> &points);  // fillpoly함수로 다각형 ROI설정 함수
void Convert_Binary(const cv::Mat &img, cv::Mat &img_binary, bool show_trackbar = false); // H,S,V 범위에 따라 이진화하는 함수  (input image, output image)
void Edge_Detect(const cv::Mat &img_gray, cv::Mat &img_edge, bool show_trackbar = false); // Gray이미지 받아서 edge 찾는 함수  (input image, output image)
void Final_Line(const cv::Mat &img_edge, std::vector<cv::Point2f> &left, std::vector<cv::Point2f> &right, cv::Scalar Lcolor = RED, cv::Scalar Rcolor = BLUE);
void Line_detect(const cv::Mat &img_edge, const cv::Mat &img_draw, bool show_trackbar = false);
cv::Point2f VanishingPoint(const std::vector<cv::Point2f> &leftLine, const std::vector<cv::Point2f> &rightLine);                  // 소실점 찾는 함수
cv::Point2f MovingAverageFilter(const cv::Point &array, std::vector<cv::Point2f> &buf = MAF_buf, size_t filter_size = MA_FILTER); // Moving Average Filter point에 적용하는 함수

cv::Point2f transformPoint(const cv::Point2f &cur, const cv::Mat &T); // 좌표계 변환 함수
void Projection(const cv::Point2f &src, cv::Point2f &dst,
                bool direction = I2W); // 픽셀좌표계-->월드좌표계 , 월드좌표계-->픽셀좌표계 변환하는 함수 (input vector, output vector, 변환 방법)
void Projection(const std::vector<cv::Point2f> &src, std::vector<cv::Point2f> &dst,
                bool direction = I2W); // 픽셀좌표계-->월드좌표계 , 월드좌표계-->픽셀좌표계 변환하는 함수 (input vector, output vector, 변환 방법)
void drawPath(const cv::Mat &img_draw, const cv::Point2f &worldP);
void drawPath(const cv::Mat &img_draw);

/*------------Fuction Expression------------*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_img = it.subscribe("/image_raw", 1, ImageCallback);
    read_pub = nh.advertise<std_msgs::Int16MultiArray>("/lookahead_Point", 1000);

    // if(!cap.isOpened()){
    //     std::cout << "Can't open the video!";
    // }

    ROS_INFO("Hello World!");
    ros::spin();
    // while (ros::ok())
    // {
    //     if (pubflag)
    //         read_pub.publish(lookahead_point);
    //     ros::spinOnce();
    // }
}

void ImageCallback(const sensor_msgs::Image::ConstPtr &img)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat img_bgr, img_binary, img_edge, img_line, img_contour, img_save, img_warp, img_read, img_roi;
    std::vector<cv::Point2f> srcPts, dstPts;
    cv::Point2f lRange;
    //ROS_INFO("IMAGE(%d, %d)", img->width, img->height);

    try
    {
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        img_bgr = cv_ptr->image;
        // video_ori << img_bgr;
        // cap >> img_bgr;
        // img_click = img_bgr.clone();
        if (cali_flag == 1)
        {
            // Projection(cv::Point2f{0,0.7},lRange,W2I); //128
            // Projection(cv::Point2f{0,1.0},lRange,W2I); //96
            // Projection(cv::Point2f{0,1.30},lRange,W2I); //73
            // Projection(cv::Point2f{0,1.50},lRange,W2I); //62
            // Projection(cv::Point2f{0,1.70},lRange,W2I); //52

            poly.clear();
            poly.push_back(cv::Point(0, SECTION1));              // 161
            poly.push_back(cv::Point(img_bgr.cols, SECTION1));   // 161
            poly.push_back(cv::Point(img_bgr.cols, SECTIONEND)); // 221
            poly.push_back(cv::Point(0, SECTIONEND));            //221

            roi1.clear();                                        // 화면 제일 하단 부분(120~170cm)
            roi1.push_back(cv::Point(0, SECTION2));              // 161
            roi1.push_back(cv::Point(img_bgr.cols, SECTION2));   // 161
            roi1.push_back(cv::Point(img_bgr.cols, SECTIONEND)); // 221
            roi1.push_back(cv::Point(0, SECTIONEND));            //221

            // setROI(img_bgr,img_roi,poly);
        }
        // drawPath(img_bgr);
        setROI(img_bgr, img_roi, poly);
        // video_ori << img_roi;
        Convert_Binary(img_roi, img_binary, true);
        Edge_Detect(img_binary, img_edge, true);
        Line_detect(img_edge, img_bgr, true);

        // video_line << img_comb;   // 저장할 영상 이미지

        // cv::imwrite("yellow_test.jpg", img_bgr);
        // std::cout << " save img "  << std::endl;
        cali_flag = 0;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Error to convert!");
        return;
    }
    // cv::imshow("Test_click", img_bgr); // 이미지에서 클릭한 점 좌표 알아내는것
    // cv::setMouseCallback("Test_click", MouseCallback);

    // cv::line(img_bgr,cv::Point(0,SECTION1), cv::Point(img_bgr.cols,SECTION1), RED, 1);
    // cv::line(img_bgr,cv::Point(0,SECTION2), cv::Point(img_bgr.cols,SECTION2), BLUE, 1);
    // cv::line(img_bgr,cv::Point(0,SECTION3), cv::Point(img_bgr.cols,SECTION3), GREEN, 1);
    // cv::line(img_bgr,cv::Point(0,SECTIONEND), cv::Point(img_bgr.cols,SECTIONEND), PURPLE, 1);
    // cv::imshow("Image section", img_roi);
    // cv::waitKey(10);
    ;
}

void MouseCallback(int event, int x, int y, int flags, void *userdata)
{
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        cv::Mat img_bev;
        cv::Point2f clickP = {(float)x, (float)y};
        cv::Point2f dstP;

        // checkHSV(img_click, clickP);

        std::cout << "clicked >> x = " << x << ", y = " << y << std::endl;
        Projection(clickP, dstP);
        // if(click_cnt%4 == 0){
        //     BirdEyeView(img_click, img_bev, clicked);
        //     clicked.clear();
        // }

        std::string coord = "(" + std::to_string((int)(dstP.x * 100)) + "," + std::to_string((int)(dstP.y * 100)) + ") cm";
        cv::Size size = cv::getTextSize(coord, font, fontScale, thickness, &baseLine); //text사이즈계산 함수
        cv::Point word_center;                                                         //text의 중심좌표를 word좌표와 일치시키기위한 계산식
        word_center.x = clickP.x - (size.width / 2);
        word_center.y = clickP.y + (size.height);
        cv::circle(img_click, clickP, 2, BLUE, -1);
        cv::putText(img_click, coord, word_center, font, fontScale, BLACK, thickness, 8);
        cv::resize(img_click, img_click, cv::Size(800, 600));
        // cv::imshow("check clicked", img_click);
        // cv::waitKey(1);
        click_cnt++;
    }
}

void setROI(const cv::Mat &src, cv::Mat &dst, const std::vector<cv::Point> &points)
{
    std::vector<std::vector<cv::Point>> poly(1, std::vector<cv::Point>());
    cv::Mat mask = cv::Mat::zeros(cv::Size(src.cols, src.rows), CV_8UC3);

    for (int i = 0; i < points.size(); i++)
    {
        poly[0].push_back(points[i]);
    }
    const cv::Point *pt = (const cv::Point *)cv::Mat(poly[0]).data;
    const cv::Point *polygon[1] = {pt};
    int npts[1] = {(int)points.size()};

    cv::fillPoly(mask, polygon, npts, 1, WHITE);
    // cv::polylines(mask, polygon, npts, 1, true, WHITE);

    cv::bitwise_and(src, mask, dst);
    // cv::imshow("fillpoly", dst);
    // cv::waitKey(1);
}

void setROIGray(const cv::Mat &src, cv::Mat &dst, const std::vector<cv::Point> &points)
{
    std::vector<std::vector<cv::Point>> poly(1, std::vector<cv::Point>());
    cv::Mat mask = cv::Mat::zeros(cv::Size(src.cols, src.rows), CV_8UC1);

    for (int i = 0; i < points.size(); i++)
    {
        poly[0].push_back(points[i]);
    }
    const cv::Point *pt = (const cv::Point *)cv::Mat(poly[0]).data;
    const cv::Point *polygon[1] = {pt};
    int npts[1] = {(int)points.size()};

    cv::fillPoly(mask, polygon, npts, 1, WHITE);
    // cv::polylines(mask, polygon, npts, 1, true, WHITE);

    cv::bitwise_and(src, mask, dst);
    // cv::imshow("fillpoly", dst);
    // cv::waitKey(1);
}

void Convert_Binary(const cv::Mat &img, cv::Mat &img_binary, bool show_trackbar)
{
    if (show_trackbar)
    {
        cv::createTrackbar("Low_H", "Binary", &lowH, 179);
        cv::createTrackbar("High_H", "Binary", &highH, 179);
        cv::createTrackbar("Low_S", "Binary", &lowS, 255);
        cv::createTrackbar("High_S", "Binary", &highS, 255);
        cv::createTrackbar("Low_V", "Binary", &lowV, 255);
        cv::createTrackbar("High_V", "Binary", &highV, 255);
    }

    cv::Mat img_hsv;
    cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);
    cv::inRange(img_hsv, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), img_binary);

    // Opening(침식->팽창), 작은점들 제거
    cv::erode(img_binary, img_binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(MORPH_MASK, MORPH_MASK)));
    cv::dilate(img_binary, img_binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(MORPH_MASK, MORPH_MASK)));

    // Closing(팽창->침식), 구멍 메우기
    cv::dilate(img_binary, img_binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(MORPH_MASK, MORPH_MASK)));
    cv::erode(img_binary, img_binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(MORPH_MASK, MORPH_MASK)));
    
    // video_bin << img_binary;
    // cv::imshow("Binary", img_binary);
    // cv::waitKey(1);
}

void Edge_Detect(const cv::Mat &img_gray, cv::Mat &img_edge, bool show_trackbar)
{
    if (show_trackbar)
    {
        cv::createTrackbar("Low_Threshold", "Edge", &lowTH, 300);
        cv::createTrackbar("High_Threshold", "Edge", &highTH, 300);
    }

    img_edge = img_gray.clone();
    cv::GaussianBlur(img_edge, img_edge, cv::Size(GAUSSIAN_MASK, GAUSSIAN_MASK), 0, 0);
    cv::Canny(img_edge, img_edge, lowTH, highTH);

    // video_edge << img_edge;
    // cv::imshow("Edge", img_edge);
    // cv::waitKey(1);
}

void Final_Line(const cv::Mat &img_edge, std::vector<cv::Point2f> &left, std::vector<cv::Point2f> &right, cv::Scalar Lcolor, cv::Scalar Rcolor)
{

    std::vector<cv::Vec4i> lines;

    cv::HoughLinesP(img_edge, lines, 1, CV_PI / 180., houghPTH, minLine, maxGap);

    float iw = img_edge.cols;
    float cx = img_edge.cols / 2.;
    float qx = img_edge.cols / 4.;
    float slope_threshold = 0.3;
    std::vector<cv::Vec4i> rightLines, leftLines, remainR, remainL; // 좌우 차선 분리하여 담을 벡터(선분의 시작좌표(x1,y1),끝좌표(x2,y2) 4개 데이터 들어있음)
    float finalLine[2][4];                                          // 최종 차선 담을 배열
    cv::Point rightP1, rightP2, leftP1, leftP2;

    for (int i = 0; i < lines.size(); i++)
    {
        int x1 = lines[i][0];
        int y1 = lines[i][1];
        int x2 = lines[i][2];
        int y2 = lines[i][3];

        float slope; // 선분 기울기

        if (x2 - x1 == 0)
            slope = 999.9; // 분모가 0이면 기울기 거의 무한대
        else
            slope = (y2 - y1) / (float)(x2 - x1);

        if (fabs(slope) > slope_threshold)
        { // 너무 수평인 직선들 제거
            if ((slope > 0) && (x1 > qx) && (x2 > qx))
            { //(x1>cx+qx) && (x2>cx+qx)
                // 기울기가 양수이고 화면 오른쪽에 위치하면 우측차선으로 분류
                // std::cout << "rightLines" << std::endl;
                rightLines.push_back(lines[i]);
            }
            else if ((slope < 0) && (x1 < cx + qx) && (x2 < cx + qx))
            { //(x1<cx-qx) && (x2<cx-qx)
                // 기울기가 음수이고 화면 왼쪽에 위치하면 좌측차선으로 분류
                // std::cout << "leftLines" << std::endl;
                leftLines.push_back(lines[i]);
            }
            else if ((x1 > qx) && (x2 > qx))
            {
                // 기울기는 음수이지만 화면 오른쪽에 위치한 차선
                // std::cout << "remainR" << std::endl;
                remainR.push_back(lines[i]);
            }
            else if ((x1 < cx + qx) && (x2 < cx + qx))
            {
                // 기울기는 양수이지만 화면 왼쪽에 위치한 차선
                // std::cout << "remainL" << std::endl;
                remainL.push_back(lines[i]);
            }
        }
    }
    
    // std::cout << "flagL : " << left_lflag << ",  flagR : " << right_lflag << std::endl;
    // 좌우 차선 둘 다 잘 검출된 경우(직진)
    if ((leftLines.size() != 0) && (rightLines.size() != 0))
    { // 좌우 차선이 검출 되었는지 확인
        // std::cout << "////           \\\\\\\\" << std::endl;
        // 최종차선 초기갑ㅅ 설정
        finalLine[0][0] = leftLines[0][0];
        finalLine[0][1] = leftLines[0][1];
        finalLine[0][2] = leftLines[0][2];
        finalLine[0][3] = leftLines[0][3];
        finalLine[1][0] = rightLines[0][0];
        finalLine[1][1] = rightLines[0][1];
        finalLine[1][2] = rightLines[0][2];
        finalLine[1][3] = rightLines[0][3];

        float slope, rslope = 0.0, lslope = 0.0; // 기울기 담을 변수

        for (int l = 0; l < leftLines.size(); l++)
        {
            if (leftLines[l][2] - leftLines[l][0] == 0)
                slope = 999.9;
            else
                slope = (leftLines[l][3] - leftLines[l][1]) /
                        (float)(leftLines[l][2] - leftLines[l][0]);

            if (slope < lslope)
            {
                // 왼쪽 차선 중 기울기가 가장 수직에 가까운 선분 검출
                finalLine[0][0] = leftLines[l][0];
                finalLine[0][1] = leftLines[l][1];
                finalLine[0][2] = leftLines[l][2];
                finalLine[0][3] = leftLines[l][3];

                lslope = slope;
            }
        }
        for (int r = 0; r < rightLines.size(); r++)
        {
            if (rightLines[r][2] - rightLines[r][0] == 0)
                slope = 999.9;
            else
                slope = (rightLines[r][3] - rightLines[r][1]) /
                        (float)(rightLines[r][2] - rightLines[r][0]);

            if (slope > rslope)
            {
                // 오른쪽 차선 중 기울기가 가장 수직에 가까운 선분 검출
                finalLine[1][0] = rightLines[r][0];
                finalLine[1][1] = rightLines[r][1];
                finalLine[1][2] = rightLines[r][2];
                finalLine[1][3] = rightLines[r][3];

                rslope = slope;
            }
        }

        // 검출된 차선들의 point 담기
        leftP1 = {(int)finalLine[0][0], (int)finalLine[0][1]};
        leftP2 = {(int)finalLine[0][2], (int)finalLine[0][3]};
        rightP1 = {(int)finalLine[1][0], (int)finalLine[1][1]};
        rightP2 = {(int)finalLine[1][2], (int)finalLine[1][3]};

        left.push_back(leftP1);
        left.push_back(leftP2);
        right.push_back(rightP1);
        right.push_back(rightP2);
        left_lflag = true;
        right_lflag = true;
    }
    // 왼쪽 차선 없고 오른쪽 \ 때
    else if ((leftLines.size() == 0) && (remainL.size() == 0) && (rightLines.size() != 0))
    { // 검출된 왼쪽차선이 없을 때
    // std::cout << "               \\\\\\\\" << std::endl;
        if (!((left_lflag == true) && (right_lflag == false)))
        {
            // 최종차선 초기갑ㅅ 설정
            finalLine[1][0] = rightLines[0][0];
            finalLine[1][1] = rightLines[0][1];
            finalLine[1][2] = rightLines[0][2];
            finalLine[1][3] = rightLines[0][3];

            float slope, rslope = 0.0, lslope = 0.0; // 기울기 담을 변수

            for (int r = 0; r < rightLines.size(); r++)
            {
                if (rightLines[r][2] - rightLines[r][0] == 0)
                    slope = 999.9;
                else
                    slope = (rightLines[r][3] - rightLines[r][1]) /
                            (float)(rightLines[r][2] - rightLines[r][0]);

                if (slope > rslope)
                {
                    // 오른쪽 차선 중 기울기가 가장 수직에 가까운 선분 검출
                    finalLine[1][0] = rightLines[r][0];
                    finalLine[1][1] = rightLines[r][1];
                    finalLine[1][2] = rightLines[r][2];
                    finalLine[1][3] = rightLines[r][3];

                    rslope = slope;
                }
            }

            // 검출된 오른쪽 차선의 point 담기
            rightP1 = {(int)finalLine[1][0], (int)finalLine[1][1]};
            rightP2 = {(int)finalLine[1][2], (int)finalLine[1][3]};

            cv::Point2f worldR1, worldR2;
            Projection(rightP1, worldR1);
            Projection(rightP2, worldR2);

            cv::Point2f virP1, virP2;
            virP1.x = worldR1.x - LANEWIDTH;
            virP1.y = worldR1.y;
            virP2.x = worldR2.x - LANEWIDTH;
            virP2.y = worldR2.y;

            // std::cout << "virP1 : " << virP1.x << ", " << virP1.y << std::endl;
            // std::cout << "virP2 : " << virP2.x << ", " << virP2.y << std::endl;

            cv::Point2f vir_leftP1, vir_leftP2;
            Projection(virP1, vir_leftP1, W2I);
            Projection(virP2, vir_leftP2, W2I);

            right.push_back(rightP1);
            right.push_back(rightP2);
            left.push_back(vir_leftP1);
            left.push_back(vir_leftP2);
            left_lflag = false;
            right_lflag = true;
        }
        else{
            // 최종차선 초기갑ㅅ 설정
            finalLine[0][0] = rightLines[0][0];
            finalLine[0][1] = rightLines[0][1];
            finalLine[0][2] = rightLines[0][2];
            finalLine[0][3] = rightLines[0][3];

            float slope, rslope = 0.0, lslope = 0.0; // 기울기 담을 변수

            for (int r = 0; r < rightLines.size(); r++)
            {
                if (rightLines[r][2] - rightLines[r][0] == 0)
                    slope = 999.9;
                else
                    slope = (rightLines[r][3] - rightLines[r][1]) /
                            (float)(rightLines[r][2] - rightLines[r][0]);

                if (slope > rslope)
                {
                    // 오른쪽 차선 중 기울기가 가장 수직에 가까운 선분 검출
                    finalLine[0][0] = rightLines[r][0];
                    finalLine[0][1] = rightLines[r][1];
                    finalLine[0][2] = rightLines[r][2];
                    finalLine[0][3] = rightLines[r][3];

                    rslope = slope;
                }
            }

            // 검출된 오른쪽 차선의 point 담기
            leftP1 = {(int)finalLine[0][0], (int)finalLine[0][1]};
            leftP2 = {(int)finalLine[0][2], (int)finalLine[0][3]};

            cv::Point2f worldR1, worldR2;
            Projection(leftP1, worldR1);
            Projection(leftP2, worldR2);

            cv::Point2f virP1, virP2;
            virP1.x = worldR1.x + LANEWIDTH;
            virP1.y = worldR1.y;
            virP2.x = worldR2.x + LANEWIDTH;
            virP2.y = worldR2.y;

            // std::cout << "virP1 : " << virP1.x << ", " << virP1.y << std::endl;
            // std::cout << "virP2 : " << virP2.x << ", " << virP2.y << std::endl;

            cv::Point2f vir_rightP1, vir_rightP2;
            Projection(virP1, vir_rightP1, W2I);
            Projection(virP2, vir_rightP2, W2I);

            right.push_back(vir_rightP1);
            right.push_back(vir_rightP2);
            left.push_back(leftP1);
            left.push_back(leftP2);
            
        }

        // std::cout << "vir_leftP1 : " << vir_leftP1.x << ", " << vir_leftP1.y << std::endl;
        // std::cout << "vir_leftP2 : " << vir_leftP2.x << ", " << vir_leftP2.y << std::endl;
    }
    // 왼쪽 / 오른쪽 차선 없을 때
    else if ((leftLines.size() != 0) && (rightLines.size() == 0) && (remainR.size() == 0))
    {   // 검출된 오른쪽차선이 없을 때
    // std::cout << "////      " << std::endl;
        // 최종차선 초기갑ㅅ 설정
        if (!((left_lflag == false) && (right_lflag == true)))
        {
            finalLine[0][0] = leftLines[0][0];
            finalLine[0][1] = leftLines[0][1];
            finalLine[0][2] = leftLines[0][2];
            finalLine[0][3] = leftLines[0][3];

            float slope, rslope = 0.0, lslope = 0.0; // 기울기 담을 변수

            for (int l = 0; l < leftLines.size(); l++)
            {
                if (leftLines[l][2] - leftLines[l][0] == 0)
                    slope = 999.9;
                else
                    slope = (leftLines[l][3] - leftLines[l][1]) /
                            (float)(leftLines[l][2] - leftLines[l][0]);

                if (slope < lslope)
                {
                    // 왼쪽 차선 중 기울기가 가장 수직에 가까운 선분 검출
                    finalLine[0][0] = leftLines[l][0];
                    finalLine[0][1] = leftLines[l][1];
                    finalLine[0][2] = leftLines[l][2];
                    finalLine[0][3] = leftLines[l][3];

                    lslope = slope;
                }
            }

            // 검출된 오른쪽 차선의 point 담기
            leftP1 = {(int)finalLine[0][0], (int)finalLine[0][1]};
            leftP2 = {(int)finalLine[0][2], (int)finalLine[0][3]};

            cv::Point2f worldL1, worldL2;
            Projection(leftP1, worldL1);
            Projection(leftP2, worldL2);

            cv::Point2f virP1, virP2;
            virP1.x = worldL1.x + LANEWIDTH;
            virP1.y = worldL1.y;
            virP2.x = worldL2.x + LANEWIDTH;
            virP2.y = worldL2.y;

            cv::Point2f vir_rightP1, vir_rightP2;
            Projection(virP1, vir_rightP1, W2I);
            Projection(virP2, vir_rightP2, W2I);

            right.push_back(vir_rightP1);
            right.push_back(vir_rightP2);
            left.push_back(leftP1);
            left.push_back(leftP2);
            left_lflag = true;
            right_lflag = false;
        }
        else
        {
            finalLine[1][0] = leftLines[0][0];
            finalLine[1][1] = leftLines[0][1];
            finalLine[1][2] = leftLines[0][2];
            finalLine[1][3] = leftLines[0][3];

            float slope, rslope = 0.0, lslope = 0.0; // 기울기 담을 변수

            for (int l = 0; l < leftLines.size(); l++)
            {
                if (leftLines[l][2] - leftLines[l][0] == 0)
                    slope = 999.9;
                else
                    slope = (leftLines[l][3] - leftLines[l][1]) /
                            (float)(leftLines[l][2] - leftLines[l][0]);

                if (slope < lslope)
                {
                    // 왼쪽 차선 중 기울기가 가장 수직에 가까운 선분 검출
                    finalLine[1][0] = leftLines[l][0];
                    finalLine[1][1] = leftLines[l][1];
                    finalLine[1][2] = leftLines[l][2];
                    finalLine[1][3] = leftLines[l][3];

                    lslope = slope;
                }
            }

            // 검출된 오른쪽 차선의 point 담기
            rightP1 = {(int)finalLine[1][0], (int)finalLine[1][1]};
            rightP2 = {(int)finalLine[1][2], (int)finalLine[1][3]};

            cv::Point2f worldL1, worldL2;
            Projection(rightP1, worldL1);
            Projection(rightP2, worldL2);

            cv::Point2f virP1, virP2;
            virP1.x = worldL1.x - LANEWIDTH;
            virP1.y = worldL1.y;
            virP2.x = worldL2.x - LANEWIDTH;
            virP2.y = worldL2.y;

            cv::Point2f vir_leftP1, vir_leftP2;
            Projection(virP1, vir_leftP1, W2I);
            Projection(virP2, vir_leftP2, W2I);

            right.push_back(rightP1);
            right.push_back(rightP2);
            left.push_back(vir_leftP1);
            left.push_back(vir_leftP2);
           
        }
    }
    // 왼쪽 \ 오른쪽 차선 없을 때
    else if ((leftLines.size() == 0) && (remainL.size() != 0) && (rightLines.size() == 0) && (remainR.size() == 0))
    { 
        // std::cout << "\\\\\\\\      " << std::endl;
        if (!((left_lflag == false) && (right_lflag == true)))
        {
            finalLine[0][0] = remainL[0][0];
            finalLine[0][1] = remainL[0][1];
            finalLine[0][2] = remainL[0][2];
            finalLine[0][3] = remainL[0][3];

            float slope, rslope = 0.0, lslope = 0.0; // 기울기 담을 변수

            for (int l = 0; l < remainL.size(); l++)
            {
                if (remainL[l][2] - remainL[l][0] == 0)
                    slope = 999.9;
                else
                    slope = (remainL[l][3] - remainL[l][1]) /
                            (float)(remainL[l][2] - remainL[l][0]);

                if (slope < lslope)
                {
                    // 왼쪽 차선 중 기울기가 가장 수직에 가까운 선분 검출
                    finalLine[0][0] = remainL[l][0];
                    finalLine[0][1] = remainL[l][1];
                    finalLine[0][2] = remainL[l][2];
                    finalLine[0][3] = remainL[l][3];

                    lslope = slope;
                }
            }

            leftP1 = {(int)finalLine[0][0], (int)finalLine[0][1]};
            leftP2 = {(int)finalLine[0][2], (int)finalLine[0][3]};

            cv::Point2f worldL1, worldL2;
            Projection(leftP1, worldL1);
            Projection(leftP2, worldL2);

            cv::Point2f virP1, virP2;
            virP1.x = worldL1.x + LANEWIDTH;
            virP1.y = worldL1.y;
            virP2.x = worldL2.x + LANEWIDTH;
            virP2.y = worldL2.y;

            cv::Point2f vir_rightP1, vir_rightP2;
            Projection(virP1, vir_rightP1, W2I);
            Projection(virP2, vir_rightP2, W2I);

            right.push_back(vir_rightP1);
            right.push_back(vir_rightP2);
            left.push_back(leftP1);
            left.push_back(leftP2);
            left_lflag = true;
            right_lflag = false;
        }
        else
        {
            finalLine[1][0] = remainL[0][0];
            finalLine[1][1] = remainL[0][1];
            finalLine[1][2] = remainL[0][2];
            finalLine[1][3] = remainL[0][3];

            float slope, rslope = 0.0, lslope = 0.0; // 기울기 담을 변수

            for (int l = 0; l < remainL.size(); l++)
            {
                if (remainL[l][2] - remainL[l][0] == 0)
                    slope = 999.9;
                else
                    slope = (remainL[l][3] - remainL[l][1]) /
                            (float)(remainL[l][2] - remainL[l][0]);

                if (slope < lslope)
                {
                    // 왼쪽 차선 중 기울기가 가장 수직에 가까운 선분 검출
                    finalLine[1][0] = remainL[l][0];
                    finalLine[1][1] = remainL[l][1];
                    finalLine[1][2] = remainL[l][2];
                    finalLine[1][3] = remainL[l][3];

                    lslope = slope;
                }
            }

            // 검출된 오른쪽 차선의 point 담기
            rightP1 = {(int)finalLine[1][0], (int)finalLine[1][1]};
            rightP2 = {(int)finalLine[1][2], (int)finalLine[1][3]};

            cv::Point2f worldL1, worldL2;
            Projection(rightP1, worldL1);
            Projection(rightP2, worldL2);

            cv::Point2f virP1, virP2;
            virP1.x = worldL1.x - LANEWIDTH;
            virP1.y = worldL1.y;
            virP2.x = worldL2.x - LANEWIDTH;
            virP2.y = worldL2.y;

            cv::Point2f vir_leftP1, vir_leftP2;
            Projection(virP1, vir_leftP1, W2I);
            Projection(virP2, vir_leftP2, W2I);

            right.push_back(rightP1);
            right.push_back(rightP2);
            left.push_back(vir_leftP1);
            left.push_back(vir_leftP2);
            
        }

        // std::cout << "vir_leftP1 : " << vir_leftP1.x << ", " << vir_leftP1.y << std::endl;
        // std::cout << "vir_leftP2 : " << vir_leftP2.x << ", " << vir_leftP2.y << std::endl;
    }
    // 왼쪽 차선 없고 오른쪽 / 때
    else if ((leftLines.size() == 0) && (remainL.size() == 0) && (rightLines.size() == 0) && (remainR.size() != 0))
    {   
        // std::cout << "               ////" << std::endl;
        // 검출된 오른쪽차선이 없을 때
        // 최종차선 초기갑ㅅ 설정
        if (!((left_lflag == true) && (right_lflag == false)))
        {
            // 최종차선 초기갑ㅅ 설정
            finalLine[1][0] = remainR[0][0];
            finalLine[1][1] = remainR[0][1];
            finalLine[1][2] = remainR[0][2];
            finalLine[1][3] = remainR[0][3];

            float slope, rslope = 0.0, lslope = 0.0; // 기울기 담을 변수

            for (int r = 0; r < remainR.size(); r++)
            {
                if (remainR[r][2] - remainR[r][0] == 0)
                    slope = 999.9;
                else
                    slope = (remainR[r][3] - remainR[r][1]) /
                            (float)(remainR[r][2] - remainR[r][0]);

                if (slope > rslope)
                {
                    // 오른쪽 차선 중 기울기가 가장 수직에 가까운 선분 검출
                    finalLine[1][0] = remainR[r][0];
                    finalLine[1][1] = remainR[r][1];
                    finalLine[1][2] = remainR[r][2];
                    finalLine[1][3] = remainR[r][3];

                    rslope = slope;
                }
            }

            // 검출된 오른쪽 차선의 point 담기
            rightP1 = {(int)finalLine[1][0], (int)finalLine[1][1]};
            rightP2 = {(int)finalLine[1][2], (int)finalLine[1][3]};

            cv::Point2f worldR1, worldR2;
            Projection(rightP1, worldR1);
            Projection(rightP2, worldR2);

            cv::Point2f virP1, virP2;
            virP1.x = worldR1.x - LANEWIDTH;
            virP1.y = worldR1.y;
            virP2.x = worldR2.x - LANEWIDTH;
            virP2.y = worldR2.y;

            // std::cout << "virP1 : " << virP1.x << ", " << virP1.y << std::endl;
            // std::cout << "virP2 : " << virP2.x << ", " << virP2.y << std::endl;

            cv::Point2f vir_leftP1, vir_leftP2;
            Projection(virP1, vir_leftP1, W2I);
            Projection(virP2, vir_leftP2, W2I);

            right.push_back(rightP1);
            right.push_back(rightP2);
            left.push_back(vir_leftP1);
            left.push_back(vir_leftP2);
            left_lflag = false;
            right_lflag = true;
        }
        else{
            // 최종차선 초기갑ㅅ 설정
            finalLine[0][0] = remainR[0][0];
            finalLine[0][1] = remainR[0][1];
            finalLine[0][2] = remainR[0][2];
            finalLine[0][3] = remainR[0][3];

            float slope, rslope = 0.0, lslope = 0.0; // 기울기 담을 변수

            for (int r = 0; r < remainR.size(); r++)
            {
                if (remainR[r][2] - remainR[r][0] == 0)
                    slope = 999.9;
                else
                    slope = (remainR[r][3] - remainR[r][1]) /
                            (float)(remainR[r][2] - remainR[r][0]);

                if (slope > rslope)
                {
                    // 오른쪽 차선 중 기울기가 가장 수직에 가까운 선분 검출
                    finalLine[0][0] = remainR[r][0];
                    finalLine[0][1] = remainR[r][1];
                    finalLine[0][2] = remainR[r][2];
                    finalLine[0][3] = remainR[r][3];

                    rslope = slope;
                }
            }

            // 검출된 오른쪽 차선의 point 담기
            leftP1 = {(int)finalLine[0][0], (int)finalLine[0][1]};
            leftP2 = {(int)finalLine[0][2], (int)finalLine[0][3]};

            cv::Point2f worldR1, worldR2;
            Projection(leftP1, worldR1);
            Projection(leftP2, worldR2);

            cv::Point2f virP1, virP2;
            virP1.x = worldR1.x + LANEWIDTH;
            virP1.y = worldR1.y;
            virP2.x = worldR2.x + LANEWIDTH;
            virP2.y = worldR2.y;

            // std::cout << "virP1 : " << virP1.x << ", " << virP1.y << std::endl;
            // std::cout << "virP2 : " << virP2.x << ", " << virP2.y << std::endl;

            cv::Point2f vir_rightP1, vir_rightP2;
            Projection(virP1, vir_rightP1, W2I);
            Projection(virP2, vir_rightP2, W2I);

            right.push_back(vir_rightP1);
            right.push_back(vir_rightP2);
            left.push_back(leftP1);
            left.push_back(leftP2);
            
        }
    }
    else
    {
        // std::cout << std::endl;
        left_lflag = false;
        right_lflag = false;
        // std::cout << "No Line Detected!" << std::endl;
    }

    if ((right.size() != 0) && (left.size() != 0))
    {
        // 차선 그리기
        cv::line(img_comb, left[0], left[1], Lcolor, 2, 8);
        cv::line(img_comb, right[0], right[1], Rcolor, 2, 8);

    }

    
}

void Line_detect(const cv::Mat &img_edge, const cv::Mat &img_draw, bool show_trackbar)
{
    img_comb = img_draw.clone();
    cv::Mat img_roi1, img_roi2, img_roi3;
    std::vector<cv::Point2f> L1, L2, L3, R1, R2, R3;
    cv::Point cp1, vp1, vp2, vp3;
    cv::Point2f s1, s2, s3;
    int cx = img_comb.cols / 2.;
    int qx = cx / 2.;
    setROIGray(img_edge, img_roi1, roi1);

    bool sec1_okay = 0;

    if (show_trackbar)
    {
        cv::createTrackbar("HoughLinesP_Threshold", "Line_detect", &houghPTH, 179);
        cv::createTrackbar("minLineLength", "Line_detect", &minLine, 179);
        cv::createTrackbar("maxLineGap", "Line_detect", &maxGap, 179);
    }

    L1.clear();
    R1.clear();
    Final_Line(img_roi1, L1, R1);

    if ((L1.size() != 0) && (R1.size() != 0))
    {
        float L1Alpha, R1Alpha;
        float L1p, R1p;
        if ((L1[1].x - L1[0].x) == 0)
            L1p = L1[0].x;
        else
        {
            L1Alpha = (L1[1].y - L1[0].y) / (float)(L1[1].x - L1[0].x);
            float L1Beta = L1[1].y - L1Alpha * L1[1].x;
            L1p = (SECTION1 - L1Beta) / L1Alpha;
        }
        if ((R1[1].x - R1[0].x) == 0)
            R1p = R1[0].x;
        else
        {
            R1Alpha = (R1[1].y - R1[0].y) / (float)(R1[1].x - R1[0].x);
            float R1Beta = R1[1].y - R1Alpha * R1[1].x;
            R1p = (SECTION1 - R1Beta) / R1Alpha;
        }

        cv::Point bottom = MovingAverageFilter({L1p, R1p}, BOTTOM_buf);
        L1p = bottom.x;
        R1p = bottom.y;

        // std::string coord = "L " + std::to_string((int)L1p) + ", R " + std::to_string((int)R1p);
        // cv::Size size = cv::getTextSize(coord, font, fontScale, thickness, &baseLine);	//text사이즈계산 함수
        // cv::Point word_center;	//text의 중심좌표를 word좌표와 일치시키기위한 계산식
        // word_center.x = (img_comb.cols/2) - (size.width / 2);
        // word_center.y = (img_comb.rows-30) + (size.height);
        // cv::putText(img_comb, coord, word_center, font, fontScale, BLACK, thickness, 8);

        if ((L1p < cx) && (R1p > cx))
        {
            cp1 = {(int)((L1p + R1p) / 2), SECTION1};
            cp1 = MovingAverageFilter(cp1); // 필터링
            vp1 = VanishingPoint(L1, R1);   // 최종 차선으로 소실점 검출
            sec1_okay = 1;
        }
    }

    if (sec1_okay)
    {
        if ((vp1.x - cp1.x) == 0)
            s1 = {(float)cp1.x, LD};
        else
        {
            float S1Alpha = (vp1.y - cp1.y) / (float)(vp1.x - cp1.x);
            float S1Beta = vp1.y - S1Alpha * vp1.x;
            float S1p = (LD - S1Beta) / S1Alpha;
            s1 = {S1p, LD};
        }
        if ((s1.x > 0) && (s1.x < img_comb.cols))
        {
            s1 = MovingAverageFilter(s1, SEC1_buf); // 필터링
            old_s1 = s1;
            // cv::line(img_comb, cp1, s1, BLACK, 2, 8);
            // cv::circle(img_comb, cp1, 5, PURPLE, -1);
            cv::circle(img_comb, s1, 5, BLACK, -1);
            cv::circle(img_comb, s1, 5, RED, -1);
            cv::Point2f wp;
            Projection(s1, wp); // Lookahead Point 실제 월드 좌표로 변환[m]
            old_wp = wp;
            std::string coord = "(" + std::to_string((int)(wp.x * 100)) + "," + std::to_string((int)(wp.y * 100)) + ") cm";
            cv::Size size = cv::getTextSize(coord, font, fontScale, thickness, &baseLine); //text사이즈계산 함수
            cv::Point word_center;                                                         //text의 중심좌표를 word좌표와 일치시키기위한 계산식
            word_center.x = s1.x - (size.width / 2);
            word_center.y = s1.y + (size.height);
            cv::putText(img_comb, coord, word_center, font, fontScale, BLACK, thickness, 8);

            lookahead_point.data.clear();
            lookahead_point.data.push_back((int)(wp.x * 100));
            lookahead_point.data.push_back((int)(wp.y * 100));
            pubflag = true;
            read_pub.publish(lookahead_point);
            std::cout << "x : " << (int)(wp.x * 100) << ", y : " << (int)(wp.y * 100) << std::endl;
        }
    }
    else
    {
        cv::circle(img_comb, old_s1, 5, RED, -1);
        std::string coord = "(" + std::to_string((int)(old_wp.x * 100)) + "," + std::to_string((int)(old_wp.y * 100)) + ") cm";
        cv::Size size = cv::getTextSize(coord, font, fontScale, thickness, &baseLine); //text사이즈계산 함수
        cv::Point word_center;                                                         //text의 중심좌표를 word좌표와 일치시키기위한 계산식
        word_center.x = old_s1.x - (size.width / 2);
        word_center.y = old_s1.y + (size.height);
        cv::putText(img_comb, coord, word_center, font, fontScale, BLACK, thickness, 8);
    }

    // cv::imshow("Line_detect", img_comb);
    // // video_line << img_comb;   // 저장할 영상 이미지
    // cv::waitKey(1);
}

cv::Point2f VanishingPoint(const std::vector<cv::Point2f> &leftLine, const std::vector<cv::Point2f> &rightLine)
{
    cv::Point2f vp;
    float rAlpha = (rightLine[1].y - rightLine[0].y) / (float)(rightLine[1].x - rightLine[0].x);
    float rBeta = rightLine[1].y - rAlpha * rightLine[1].x;
    float lAlpha = (leftLine[1].y - leftLine[0].y) / (float)(leftLine[1].x - leftLine[0].x);
    float lBeta = leftLine[1].y - lAlpha * leftLine[1].x;

    vp.x = ((rBeta - lBeta) / (lAlpha - rAlpha));
    vp.y = (lAlpha * vp.x + lBeta);

    return vp;
}

cv::Point2f MovingAverageFilter(const cv::Point &array, std::vector<cv::Point2f> &buf, size_t filter_size)
{
    float sumx = 0.0, sumy = 0.0;
    float avgx, avgy;

    if (buf.size() == 0)
    {
        // 벡터크기가 0이면(배열이 비었으면) --> 첫 데이터로 다 채우기
        for (int i = 0; i < filter_size; i++)
        {
            buf.push_back(array);
        }
        return array;
    }
    else
    {
        buf.push_back(array);
        for (int i = 0; i < filter_size; i++)
        {
            sumx += buf[buf.size() - (1 + i)].x;
            sumy += buf[buf.size() - (1 + i)].y;
        }
        avgx = sumx / (float)filter_size;
        avgy = sumy / (float)filter_size;
        return cv::Point2f(avgx, avgy);
    }
}

cv::Point2f transformPoint(const cv::Point2f &cur, const cv::Mat &T)
{
    cv::Point2f tPoint;

    tPoint.x = cur.x * T.at<double>(0, 0) + cur.y * T.at<double>(0, 1) + T.at<double>(0, 2);
    tPoint.y = cur.x * T.at<double>(1, 0) + cur.y * T.at<double>(1, 1) + T.at<double>(1, 2);
    float z = cur.x * T.at<double>(2, 0) + cur.y * T.at<double>(2, 1) + T.at<double>(2, 2);

    tPoint.x /= z;
    tPoint.y /= z;

    return tPoint;
}

void Projection(const cv::Point2f &src, cv::Point2f &dst, bool direction)
{
    cv::Mat img_warp;
    cv::Point2f p;
    std::vector<cv::Point2f> imagePoints;
    std::vector<cv::Point2f> objectPoints;

    imagePoints.push_back(cv::Point2f(37, 104));
    imagePoints.push_back(cv::Point2f(139, 103));
    imagePoints.push_back(cv::Point2f(124, 62));
    imagePoints.push_back(cv::Point2f(51, 63));

    objectPoints.push_back(cv::Point2f(-0.3, 1.22));
    objectPoints.push_back(cv::Point2f(0.3, 1.22));
    objectPoints.push_back(cv::Point2f(0.3, 1.82));
    objectPoints.push_back(cv::Point2f(-0.3, 1.82));

    cv::Mat img2World = cv::getPerspectiveTransform(imagePoints, objectPoints);
    cv::Mat world2Image = img2World.inv();

    if (direction)
    {
        p = transformPoint(src, img2World);
        // std::cout << "Real : " << p.x << ", " << p.y << std::endl;
    }
    else
    {
        p = transformPoint(src, world2Image);
        // std::cout << "IMG : " << p.x << ", " << p.y << std::endl;
    }
    dst = p;
}

void Projection(const std::vector<cv::Point2f> &src, std::vector<cv::Point2f> &dst, bool direction)
{
    cv::Mat img_warp;
    std::vector<cv::Point2f> imagePoints;
    std::vector<cv::Point2f> objectPoints;

    imagePoints.push_back(cv::Point2f(37, 104));
    imagePoints.push_back(cv::Point2f(139, 103));
    imagePoints.push_back(cv::Point2f(124, 62));
    imagePoints.push_back(cv::Point2f(51, 63));

    objectPoints.push_back(cv::Point2f(-0.3, 1.22));
    objectPoints.push_back(cv::Point2f(0.3, 1.22));
    objectPoints.push_back(cv::Point2f(0.3, 1.82));
    objectPoints.push_back(cv::Point2f(-0.3, 1.82));

    cv::Mat img2World = cv::getPerspectiveTransform(imagePoints, objectPoints);
    cv::Mat world2Image = img2World.inv();

    for (int i = 0; i < src.size(); i++)
    {
        cv::Point2f p;
        if (direction)
            p = transformPoint(src[i], img2World);
        else
            p = transformPoint(src[i], world2Image);
        dst.push_back(p);
        // std::cout << "Real [" << i << "] (" << p.x << ", " << p.y << std::endl;
    }
}
