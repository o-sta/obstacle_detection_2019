#include <obstacle_detection_2019/darknetImgDebug.h>
// #include "opencv2/opencv.hpp"

int main(int argc,char **argv){
	ros::init(argc,argv,"obstacle_detection_2019_darknet");
	ROS_INFO_STREAM("debug setParam");
    darknetImgDebug ccd;
	ros::spin();
	return 0;
}