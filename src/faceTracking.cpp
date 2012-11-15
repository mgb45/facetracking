#include "faceTrackerNew.h"

int main( int argc, char** argv )
{
	ros::init(argc, argv, "faceTracking");
	
	FaceTracker *tracker = new FaceTracker();
	
	ros::spin();
	delete tracker;	
	return 0;
}
