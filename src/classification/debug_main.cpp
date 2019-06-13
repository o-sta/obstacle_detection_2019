#include <ros/ros.h>
#include <obstacle_detection_2019/classification.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"obstacle_detection_2019_cc");
	
    classificationClass cc; //pan_tilt_control_instance
	ROS_INFO("creatMapIndex");
	cc.creatMapIndex();
	while(ros::ok())
	{
		// ROS_INFO("subscribeSensorDataCamera");
		// cc.subscribeSensorDataCamera();
		// ROS_INFO("sortSensorData");
		// cc.sortSensorData();
		// ROS_INFO("compressSensorData");
		// cc.compressSensorData();
	}
    // ros::spin();
	return 0;
}