#include <ros/ros.h>
#include <obstacle_detection_2019/imageMatching.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"obstacle_detection_2019_im");
	
    imageMatchingClass imc; //pan_tilt_control_instance
	// while(ros::ok())
	// {
	// }
    ros::spin();
	return 0;
}