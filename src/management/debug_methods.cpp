#include<obstacle_detection_2019/management.h>

void managementClass::delayTime(){
    ROS_INFO_STREAM("now - depthTime = "<<ros::Duration(ros::Time::now() - depthImage.header.stamp).toSec());
}
void managementClass::culcDifTime(){
    ROS_INFO_STREAM("difTime = "<<ros::Duration(depthImage.header.stamp-rgbImage.header.stamp).toSec());
}