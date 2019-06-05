#include <ros/ros.h>
#include <obstacle_detection_2019/convCamData.h>
#include <iostream>

int main(int argc,char **argv){
	ros::init(argc,argv,"obstacle_detection_2019_rg");
	
    convCamDataClass ccd; //pan_tilt_control_instance
	ROS_INFO_STREAM("convCamDataClass instance generated.\n");
	ros::Rate loop_rate(60);
	ros::Time ransac_time = ros::Time::now();
	while (ros::ok()) {
		ccd.subscribeSensorData();
		if (ros::Time::now() - ransac_time > ros::Duration(1)){
			if (ccd.is_SensorData_subscribed()){
				ROS_INFO_STREAM("RANSAC start.\n");
				ccd.groundEstimationRANSAC();
				ransac_time = ros::Time::now();
			}else{
				ROS_INFO_STREAM("Sensor data not exist.\n");
			}
		}
		loop_rate.sleep();
	}
	return 0;
}