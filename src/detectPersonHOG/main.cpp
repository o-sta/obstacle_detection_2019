#include <ros/ros.h>
#include <obstacle_detection_2019/detectPersonHOG.h>
// #include "opencv2/opencv.hpp"

int main(int argc,char **argv){
	ros::init(argc,argv,"obstacle_detection_2019_gr");
    detectPersonHOG ccd;
	ros::spin();
	return 0;
}
