#include <ros/ros.h>
#include <obstacle_detection_2019/darknetImg.h>
// #include "opencv2/opencv.hpp"

int main(int argc,char **argv){
	ros::init(argc,argv,"obstacle_detection_2019_darknet");
    darknetImg ccd;
	ros::spin();
	return 0;
}
