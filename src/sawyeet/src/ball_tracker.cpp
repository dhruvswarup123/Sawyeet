/******************************************
Kalman filter and tracker from 
https://www.myzhar.com/blog/tutorials/tutorial-opencv-ball-tracker-using-kalman-filter/
*****************************************/

#include "geometry_msgs/Point.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <vector>

#define TENNIS

#ifdef TENNIS
    #define COLOR_MIN 20, 80, 45
    #define COLOR_MAX 45, 255, 255
#endif
#ifdef ORANGE
    #define COLOR_MIN 0, 150, 50
    #define COLOR_MAX 35, 255, 255
#endif

using namespace std;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace message_filters;

// >>>> Globals for kalman tracking
int stateSize = 8;
int measSize = 5;
int contrSize = 0;

unsigned int type = CV_32F;

double ticks = 0;
bool found = false;
int notFoundCount = 0;

cv::KalmanFilter kf;
cv::Mat state;
cv::Mat meas;
float prev_measured_z = 0;

// <<<< Globals for kalman tracking

ros::Publisher chatter_pub;

void chatterCallback(const Image::ConstPtr& image_msg, const CameraInfo::ConstPtr& camerainfo_msg, const Image::ConstPtr& depths_msg){
    // >>>> Cam mat, depths
    Eigen::Matrix3f camera_matrix;
    camera_matrix(0, 0) = camerainfo_msg->K[0];
    camera_matrix(0, 1) = camerainfo_msg->K[1];
    camera_matrix(0, 2) = camerainfo_msg->K[2];
    camera_matrix(1, 0) = camerainfo_msg->K[3];
    camera_matrix(1, 1) = camerainfo_msg->K[4];
    camera_matrix(1, 2) = camerainfo_msg->K[5];
    camera_matrix(2, 0) = camerainfo_msg->K[6];
    camera_matrix(2, 1) = camerainfo_msg->K[7];
    camera_matrix(2, 2) = camerainfo_msg->K[8];

    cv::Mat depths;
    cv_bridge::CvImagePtr cv_depths_ptr;
    
    try {
        cv_depths_ptr = cv_bridge::toCvCopy(depths_msg, sensor_msgs::image_encodings::TYPE_8UC1);
        cv_depths_ptr->image.copyTo(depths);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    // <<<< Cam mat, depths
    
    double precTick = ticks;
    ticks = (depths_msg->header).stamp.toSec();
    double dT = ticks - precTick;

    // >>>> Frame acquisition
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat frame, res;
    
    try{
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_ptr->image.copyTo(frame);
    frame.copyTo(res);
    // <<<< Frame acquisition

    if (found)
    {
        // >>>> Matrix A
        kf.transitionMatrix.at<float>(3) = dT;
        kf.transitionMatrix.at<float>(12) = dT;
        kf.transitionMatrix.at<float>(21) = dT;
        // <<<< Matrix A

        state = kf.predict();
        
        cv::Rect predRect;
        predRect.width = state.at<float>(6);
        predRect.height = state.at<float>(7);
        predRect.x = state.at<float>(0) - predRect.width / 2;
        predRect.y = state.at<float>(1) - predRect.height / 2;

        cv::Point center;
        center.x = state.at<float>(0);
        center.y = state.at<float>(1);
        float pred_depth = state.at<float>(2);

        cv::circle(res, center, 2, CV_RGB(255,0,0), -1);
        cv::rectangle(res, predRect, CV_RGB(255,0,0), 2);


        // >>>> Homog pixels
        Eigen::Matrix<float, 3, 1> homog_pixels;
        homog_pixels(0, 0) = center.x;
        homog_pixels(1, 0) = center.y;
        homog_pixels(2, 0) = 1;
        // <<<< Homog pixels

        // >>>> transformation from depths to realsense
        Eigen::Matrix<float, 3, 1> camera_to_realsense;
        camera_to_realsense(0, 0) = 0.0106;
        camera_to_realsense(1, 0) = 0.0175;
        camera_to_realsense(2, 0) = 0.0125;
        // <<<< transformation from depths to realsense

        // >>> 3d coords
        Eigen::Matrix<float, 3, 1> coords_3d = camera_matrix.inverse() * homog_pixels * pred_depth;
        coords_3d += camera_to_realsense;
        cout << "Prev d: "<<  prev_measured_z << endl;
        cout << "Pred d: "<<  meas.at<float>(2) << endl;

        // <<< 3d coords

        cout << "Center: " << center << endl;
        printf("3d coords: [%0.3f, %0.3f, %0.3f]\n", coords_3d[0], coords_3d[1], coords_3d[2]);

        Point coords_3d_msg;

        coords_3d_msg.x = coords_3d[0];
        coords_3d_msg.y = coords_3d[1];
        coords_3d_msg.z = coords_3d[2];
        
        chatter_pub.publish(coords_3d_msg);
    }

    // >>>>> Noise smoothing
    cv::Mat blur;
    cv::GaussianBlur(frame, blur, cv::Size(5, 5), 3.0, 3.0);
    // <<<<< Noise smoothing

    // >>>>> HSV conversion
    cv::Mat frmHsv;
    cv::cvtColor(blur, frmHsv, cv::COLOR_BGR2HSV);
    // <<<<< HSV conversion

    // >>>>> Color Thresholding
    // Note: change parameters for different colors
    cv::Mat rangeRes = cv::Mat::zeros(frame.size(), CV_8UC1);

    cv::inRange(frmHsv, cv::Scalar(COLOR_MIN),
                        cv::Scalar(COLOR_MAX), rangeRes);

    // <<<<< Color Thresholding

    // >>>>> Improving the result
    cv::erode(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
    // cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
    // <<<<< Improving the result

    // Thresholding viewing
    cv::imshow("Threshold", rangeRes);

    // >>>>> Contours detection
    vector<vector<cv::Point> > contours;
    cv::findContours(rangeRes, contours, cv::RETR_EXTERNAL,
                        cv::CHAIN_APPROX_NONE);
    // <<<<< Contours detection

    // >>>>> Filtering
    vector<vector<cv::Point> > balls;
    vector<cv::Rect> ballsBox;
    for (size_t i = 0; i < contours.size(); i++)
    {
        cv::Rect bBox;
        bBox = cv::boundingRect(contours[i]);

        float ratio = (float) bBox.width / (float) bBox.height;
        if (ratio > 1.0f)
            ratio = 1.0f / ratio;

        // Searching for a bBox almost square
        if (ratio > 0.75 && bBox.area() >= 400)
        {
            balls.push_back(contours[i]);
            ballsBox.push_back(bBox);
        }
    }
    // <<<<< Filtering

    // >>>>> Detection result
    for (size_t i = 0; i < balls.size(); i++)
    {
        cv::drawContours(res, balls, i, CV_RGB(20,150,20), -1);
    }
    // <<<<< Detection result

    // >>>>> Kalman Update
    if (balls.size() == 0)
    {
        notFoundCount++;
        cout << "notFoundCount:" << notFoundCount << endl;
        if( notFoundCount >= 15)
        {
            found = false;
        }
    }
    else
    {
        notFoundCount = 0;

        meas.at<float>(0) = ballsBox[0].x + ballsBox[0].width / 2;
        meas.at<float>(1) = ballsBox[0].y + ballsBox[0].height / 2;

        int temp_total_points = 0;
        float temp_depth_sum = 0;

        for (int i = 0; i < balls[0].size(); i++){
            float temp_depth = depths.at<float>(balls[0][i].x, balls[0][i].y) * 0.001;
            if ((temp_depth != 0) && (temp_depth <= 10.0)){
                temp_total_points++;
                temp_depth_sum += temp_depth;
            }
        }

        float temp_depth_avg = temp_depth_sum/temp_total_points;
        if (temp_total_points != 0 && temp_depth_avg > 0.0001){
            meas.at<float>(2) = temp_depth_avg;
            prev_measured_z = meas.at<float>(2);
        }
        else {
            meas.at<float>(2) = 0;
        }

        meas.at<float>(3) = (float)ballsBox[0].width;
        meas.at<float>(4) = (float)ballsBox[0].height;

        if (!found) // First detection!
        {
            // >>>> Initialization
            kf.errorCovPre.at<float>(0) = 1;
            kf.errorCovPre.at<float>(9) = 1;
            kf.errorCovPre.at<float>(18) = 1;
            kf.errorCovPre.at<float>(27) = 1;
            kf.errorCovPre.at<float>(36) = 1;
            kf.errorCovPre.at<float>(45) = 1;
            kf.errorCovPre.at<float>(54) = 1;
            kf.errorCovPre.at<float>(63) = 1;

            state.at<float>(0) = meas.at<float>(0);
            state.at<float>(1) = meas.at<float>(1);
            state.at<float>(2) = meas.at<float>(2);
            state.at<float>(3) = 0;
            state.at<float>(4) = 0;
            state.at<float>(5) = 0;
            state.at<float>(6) = meas.at<float>(3);
            state.at<float>(7) = meas.at<float>(4);
            // <<<< Initialization

            kf.statePost = state;
            
            found = true;
        }
        else
            kf.correct(meas); // Kalman Correction
    }
    // <<<<< Kalman Update

    // Final result
    cv::imshow("Tracking", res);
    // User key
    cv::waitKey(1);
}

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "ball_tracker");
    ros::NodeHandle nh;

    // ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);

    message_filters::Subscriber<Image> image_sub(nh, "/camera/color/image_raw", 1);
    message_filters::Subscriber<CameraInfo> info_sub(nh, "/camera/color/camera_info", 1);
    // message_filters::Subscriber<Image> points_sub(nh, "/camera/aligned_depth_to_color/image_raw", 1);
    message_filters::Subscriber<Image> points_sub(nh, "/camera/depth/image_rect_raw", 1);

    chatter_pub = nh.advertise<geometry_msgs::Point>("/sawyeet/ball_coords", 5);

    TimeSynchronizer<Image, CameraInfo, Image> sync(image_sub, info_sub, points_sub, 10);
    sync.registerCallback(boost::bind(&chatterCallback, _1, _2, _3));

    // >>>> Kalman Filter
    
    kf = cv::KalmanFilter(stateSize, measSize, contrSize, type);

    state = cv::Mat(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
    meas = cv::Mat(measSize, 1, type);    // [z_x,z_y,z_w,z_h]

    // Transition State Matrix A
    cv::setIdentity(kf.transitionMatrix);

    // Measure Matrix H
    //   x y z v v v w h
    // [ 1 0 0 0 0 0 0 0] x
    // [ 0 1 0 0 0 0 0 0] y
    // [ 0 0 1 0 0 0 0 0] z
    // [ 0 0 0 0 0 0 1 0] w
    // [ 0 0 0 0 0 0 0 1] h
    kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
    kf.measurementMatrix.at<float>(0) = 1.0f;
    kf.measurementMatrix.at<float>(9) = 1.0f;
    kf.measurementMatrix.at<float>(18) = 1.0f;
    kf.measurementMatrix.at<float>(30) = 1.0f;
    kf.measurementMatrix.at<float>(39) = 1.0f;

    // cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
    kf.processNoiseCov.at<float>(0) = 1e-2;
    kf.processNoiseCov.at<float>(9) = 1e-2;
    kf.processNoiseCov.at<float>(18) = 1e-2;
    kf.processNoiseCov.at<float>(27) = 5.0f;
    kf.processNoiseCov.at<float>(36) = 5.0f;
    kf.processNoiseCov.at<float>(45) = 5.0f;
    kf.processNoiseCov.at<float>(54) = 1e-2;
    kf.processNoiseCov.at<float>(63) = 1e-2;

    // Measures Noise Covariance Matrix R
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
    // <<<< Kalman Filter

    ros::spin();

    return 0;
}