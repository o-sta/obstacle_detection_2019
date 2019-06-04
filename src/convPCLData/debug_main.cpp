#include <ros/ros.h>
#include <obstacle_detection_2019/convPCLData.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"obstacle_detection_2019_pcl");
	
    convPCLDataClass cpcldc; //pan_tilt_control_instance
	while(ros::ok())
	{
		ROS_INFO("subscribeSensorData");
		cpcldc.subscribeSensorData();
		ROS_INFO("convCloud2toPCL");
		cpcldc.convCloud2toPCL();
		ROS_INFO("convPCLtoImages");
		cpcldc.convPCLtoImages();
		ROS_INFO("publish images");
		cpcldc.publishRgbCamData();
		cpcldc.publishDepthCamData();
	}
	return 0;
}