#include <ros/ros.h>
#include <obstacle_detection_2019/velocityEstimation.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"obstacle_detection_2019_estimate");
	
	// ROS_INFO("velocityEstimation define");
    velocityEstimation ec; //
    ros::spin();
	return 0;
}