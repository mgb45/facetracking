#ifndef __FACETRACKER
#define __FACETRACKER

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include <iostream>
#include <opencv/cv.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/video/tracking.hpp"
#include <opencv2/features2d/features2d.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/Image.h"


class face 
{
	public:
		face(cv::Rect roi_in, double dt);
		~face();
		cv::Rect roi;
		int views;
		bool visible;
		void predictPos(double dt);
		void updatePos(float x, float y);
	private:
		cv::KalmanFilter tracker;
};

class FaceTracker
{
	public:
		FaceTracker();
		~FaceTracker();
		std::vector<face> getVisibleFaces();
	private:
		void updateFaces(std::vector<cv::Rect> roi, cv::Mat input, double dt);
		std::vector<face> faces;
		
		//Face detect
		std::vector<cv::Rect> findFaces(cv::Mat frame);
		/** Global variables */
		std::string face_cascade_name;
		cv::CascadeClassifier face_cascade;
		int faceThresh;
		
		ros::NodeHandle nh;
		double dtime;
		image_transport::Publisher pub;
		image_transport::Subscriber sub;
		void imageCallback(const sensor_msgs::ImageConstPtr& msg);
};

#endif
