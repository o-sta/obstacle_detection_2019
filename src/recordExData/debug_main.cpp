#include <ros/ros.h>
#include <obstacle_detection_2019/recordExData.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"obstacle_detection_2019_red");
	
	ROS_INFO("recordExData define");
    recordExData red; //
    ros::spin();
	return 0;
}