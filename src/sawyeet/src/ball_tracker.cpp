/******************************************
 * Kalman filter and tracker from https://www.myzhar.com/blog/tutorials/tutorial-opencv-ball-tracker-using-kalman-filter/
 ******************************************/

#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "geometry_msgs/Point.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>

#include <iostream>
#include <vector>

#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace message_filters;

// >>>> Globals for kalman tracking
int stateSize = 6;
int measSize = 4;
int contrSize = 0;

unsigned int type = CV_32F;

double ticks = 0;
bool found = false;
int notFoundCount = 0;

cv::KalmanFilter kf;
cv::Mat state;
cv::Mat meas;
// <<<< Globals for kalman tracking

void chatterCallback(const Image::ConstPtr& image_msg, const CameraInfo::ConstPtr& camerainfo_msg, const Image::ConstPtr& points_msg){
    double precTick = ticks;
    ticks = (points_msg->header).stamp.toSec();

    // TODO: Get time difference here
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
        kf.transitionMatrix.at<float>(2) = dT;
        kf.transitionMatrix.at<float>(9) = dT;
        // <<<< Matrix A

        state = kf.predict();

        cv::Rect predRect;
        predRect.width = state.at<float>(4);
        predRect.height = state.at<float>(5);
        predRect.x = state.at<float>(0) - predRect.width / 2;
        predRect.y = state.at<float>(1) - predRect.height / 2;

        cv::Point center;
        center.x = state.at<float>(0);
        center.y = state.at<float>(1);
        cv::circle(res, center, 2, CV_RGB(255,0,0), -1);

        cv::rectangle(res, predRect, CV_RGB(255,0,0), 2);
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

    //TODO: Add if block for color?
    cv::inRange(frmHsv, cv::Scalar(20, 80, 45),
                        cv::Scalar(45, 255, 255), rangeRes);
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
        if( notFoundCount >= 100 )
        {
            found = false;
        }
    }
    else
    {
        notFoundCount = 0;

        meas.at<float>(0) = ballsBox[0].x + ballsBox[0].width / 2;
        meas.at<float>(1) = ballsBox[0].y + ballsBox[0].height / 2;
        meas.at<float>(2) = (float)ballsBox[0].width;
        meas.at<float>(3) = (float)ballsBox[0].height;

        if (!found) // First detection!
        {
            // >>>> Initialization
            kf.errorCovPre.at<float>(0) = 1; // px
            kf.errorCovPre.at<float>(7) = 1; // px
            kf.errorCovPre.at<float>(14) = 1;
            kf.errorCovPre.at<float>(21) = 1;
            kf.errorCovPre.at<float>(28) = 1; // px
            kf.errorCovPre.at<float>(35) = 1; // px

            state.at<float>(0) = meas.at<float>(0);
            state.at<float>(1) = meas.at<float>(1);
            state.at<float>(2) = 0;
            state.at<float>(3) = 0;
            state.at<float>(4) = meas.at<float>(2);
            state.at<float>(5) = meas.at<float>(3);
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

    message_filters::Subscriber<Image> image_sub(nh, "image", 1);
    message_filters::Subscriber<CameraInfo> info_sub(nh, "camera_info", 1);
    message_filters::Subscriber<Image> points_sub(nh, "points", 1);

    TimeSynchronizer<Image, CameraInfo, Image> sync(image_sub, info_sub, points_sub, 10);
    sync.registerCallback(boost::bind(&chatterCallback, _1, _2, _3));

    // >>>> Kalman Filter
    
    kf = cv::KalmanFilter(stateSize, measSize, contrSize, type);

    state = cv::Mat(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
    meas = cv::Mat(measSize, 1, type);    // [z_x,z_y,z_w,z_h]

    // Transition State Matrix A
    cv::setIdentity(kf.transitionMatrix);

    // Measure Matrix H
    kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
    kf.measurementMatrix.at<float>(0) = 1.0f;
    kf.measurementMatrix.at<float>(7) = 1.0f;
    kf.measurementMatrix.at<float>(16) = 1.0f;
    kf.measurementMatrix.at<float>(23) = 1.0f;

    kf.processNoiseCov.at<float>(0) = 1e-2;
    kf.processNoiseCov.at<float>(7) = 1e-2;
    kf.processNoiseCov.at<float>(14) = 5.0f;
    kf.processNoiseCov.at<float>(21) = 5.0f;
    kf.processNoiseCov.at<float>(28) = 1e-2;
    kf.processNoiseCov.at<float>(35) = 1e-2;

    // Measures Noise Covariance Matrix R
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
    // <<<< Kalman Filter

    ros::spin();

    return 0;
}