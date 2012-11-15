#include "faceTrackerNew.h"

using namespace cv;
using namespace std;

face::face(cv::Rect roi_in, double dt)
{
	tracker.init(4,2,0,CV_32F);
	tracker.transitionMatrix = *(Mat_<float> (4, 4) << 1, 0, dt, 0, 0, 1, 0, dt, 0, 0, 1, 0, 0, 0, 0, 1);
	tracker.statePre.at<float>(0) = roi_in.x;
	tracker.statePre.at<float>(1) = roi_in.y;
	tracker.statePre.at<float>(2) = 0;
	tracker.statePre.at<float>(3) = 0;
	
	setIdentity(tracker.measurementMatrix);
	setIdentity(tracker.processNoiseCov, Scalar::all(80));
	tracker.processNoiseCov.at<float>(0,0) = 0;
	tracker.processNoiseCov.at<float>(1,1) = 0;
	setIdentity(tracker.measurementNoiseCov, Scalar::all(5));
	setIdentity(tracker.errorCovPost, Scalar::all(2));
}

face::~face()
{

}

void face::predictPos(double dt)
{
	tracker.transitionMatrix.at<float>(0,2) = dt;
	tracker.transitionMatrix.at<float>(1,3) = dt;
	Mat prediction = tracker.predict();
	roi.x = std::max((int)prediction.at<float>(0),0);
	roi.x = std::min(roi.x,640-roi.width);
	roi.y = std::max((int)prediction.at<float>(1),0);
	roi.y = std::min(roi.y,480-roi.height);
}

void face::updatePos(float x, float y)
{
	Mat_<float> measurement(2,1);
	measurement.at<float>(0) = x;
	measurement.at<float>(1) = y;
	Mat estimated = tracker.correct(measurement);
	roi.x = std::max((int)estimated.at<float>(0),0);
	roi.x = std::min(roi.x,640-roi.width);
	roi.y = std::max((int)estimated.at<float>(1),0);
	roi.y = std::min(roi.y,480-roi.height);
	
}

FaceTracker::FaceTracker()
{
	
	image_transport::ImageTransport it(nh); //ROS
	
	pub = it.advertise("/outputImage",1); //ROS
 	
	sub = it.subscribe("/image", 1, &FaceTracker::imageCallback,this); //ROS
	
	//Face detection
	/** Global variables */
	face_cascade_name = "/opt/ros/fuerte/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml"; 
	
	//-- 1. Load the cascades
	if (!face_cascade.load(face_cascade_name))
	{ 
		ROS_ERROR("--(!)Error loading\n");
	}
	
	faceThresh = 3;
	
	dtime = ros::Time::now().toSec();
}

FaceTracker::~FaceTracker()
{
	
}

void FaceTracker::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	ROS_INFO("Image Received");  //ROS
	namespace enc = sensor_msgs::image_encodings;
	try
	{	
		cv::Mat image = (cv_bridge::toCvCopy(msg, enc::RGB8))->image; //ROS
		std::vector<cv::Rect> faceROIs = findFaces(image);
		updateFaces(faceROIs, image, msg->header.stamp.toSec()-dtime);
		dtime = msg->header.stamp.toSec();
		
		for (int i = 0; i < faces.size(); i++)
		{
			if (faces[i].views >= faceThresh)
			{
				rectangle(image, Point(faces[i].roi.x, faces[i].roi.y), Point(faces[i].roi.x+faces[i].roi.width, faces[i].roi.y+faces[i].roi.height), Scalar(0, 0, 255), 4, 8, 0);
			}
		}
				
		cv_bridge::CvImage img2;
		img2.encoding = "rgb8";
		img2.image = image;			
		pub.publish(img2.toImageMsg());
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}	
}

std::vector<face> FaceTracker::getVisibleFaces()
{
	std::vector<face> visibleFaces;
	for (int j = 0; j < (int)faces.size(); j ++)
	{
		if (faces[j].views >= faceThresh)//(faces[j].visible && 
		{
			visibleFaces.push_back(faces[j]);
		}
	}
	return visibleFaces;
}

void FaceTracker::updateFaces(std::vector<cv::Rect> roi, cv::Mat input, double dt)
{
	for (int i = 0; i < (int)faces.size(); i++)
	{
		//ROS_INFO("Face %d: %d",i,faces[i].views);
		faces[i].views = faces[i].views-1;
		faces[i].visible = false;
		
		if (faces[i].views < 0) //prune face list
		{
			faces.erase(faces.begin()+i);
		}
		else
		{
			faces[i].predictPos(dt);
			ROS_INFO("Face %d: %d %d %d %d %d",i,faces[i].views, faces[i].roi.x, faces[i].roi.y, faces[i].roi.width, faces[i].roi.height);	
		}
	}
	
	for (int k1 = 0; k1 < (int)roi.size(); k1++)
	{
		if ((int)faces.size() == 0) // No faces yet
		{
			face new_face(roi[k1],dt);
			new_face.roi = roi[k1];
			new_face.visible = true;
			new_face.views = 1;
			faces.push_back(new_face);
			ROS_INFO ("New Face");
		}
		else // Faces already present
		{
			double minDist = 10000;
			int bestBin = -1;
			for (int i = 0; i < (int)faces.size(); i ++) // Find matching faces
			{
				double d = sqrt(pow(faces[i].roi.x - roi[k1].x,2) + pow(faces[i].roi.y - roi[k1].y,2) + pow(faces[i].roi.width - roi[k1].width,2) + pow(faces[i].roi.height - roi[k1].height,2));
				if (d < minDist)
				{
					minDist = d;
					bestBin = i;
				}
			}
			if (minDist > 80) // No matching faces - add new face
			{
				//ROS_INFO("New Face: %d %f",faces[bestBin].views,minDist);
				face new_face(roi[k1],dt);
				new_face.roi = roi[k1];
				new_face.visible = true;
				new_face.views = 1;
				faces.push_back(new_face);
				ROS_INFO ("Added Face");
			}
			else // Matching face - update current
			{
				//ROS_INFO("Old Face: %d %f",faces[bestBin].views,minDist);
				faces[bestBin].roi.height = roi[k1].height;
				faces[bestBin].roi.width = roi[k1].width;
				faces[bestBin].updatePos(roi[k1].x,roi[k1].y);
				ROS_INFO("Face %d: %d %d %d",bestBin,faces[bestBin].views, faces[bestBin].roi.x, faces[bestBin].roi.y);	
				faces[bestBin].views = faces[bestBin].views + 2;
				faces[bestBin].views = std::min(faces[bestBin].views,10);
				faces[bestBin].visible = true;
			}
		}
	}
}

std::vector<cv::Rect> FaceTracker::findFaces(cv::Mat frame)
{
	std::vector<Rect> faces;
	Mat frame_gray;
	cvtColor(frame, frame_gray, CV_BGR2GRAY);
	equalizeHist(frame_gray, frame_gray);

	//-- Detect faces
	face_cascade.detectMultiScale(frame_gray, faces, 1.3, 3, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30));

	for (int i = 0; i < faces.size(); i++)
	{
		//faceROIs.push_back(frame1(faces[i]));
		Point center(faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5);
		ellipse(frame, center, Size(faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar(255, 0, 255), 4, 8, 0);
	}
	return faces;
 }

