#include<obstacle_detection_2019/management.h>

void managementClass::delayTime(){
    ROS_INFO_STREAM("now - depthTime = "<<ros::Duration(ros::Time::now() - depthImage.header.stamp).toSec());
}
void managementClass::culcPastTime(){
    ROS_INFO_STREAM("difTime between Rgb and Depth = "<<ros::Duration(depthImage.header.stamp-rgbImage.header.stamp).toSec());
}
void managementClass::culcDifTime(const obstacle_detection_2019::synchronizedImage::ConstPtr& imageMsg){
    ROS_INFO_STREAM("difTime between neighborhood frames = "<<ros::Duration(imageMsg->rgb.header.stamp-depthImage.header.stamp).toSec());
}