#include <ros/ros.h>
#include <obstacle_detection_2019/convLRFData.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"obstacle_detection_2019_lrf");
	
    convLRFDataClass clrfdc; //pan_tilt_control_instance
	// while(ros::ok())
	// {
	// }
    ros::spin();
	return 0;
}