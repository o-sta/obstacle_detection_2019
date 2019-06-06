#include<obstacle_detection_2019/management.h>

//subscribe
void managementClass::subscribeData(){//データ受信
	queue.callOne(ros::WallDuration(1));
}
void managementClass::callback(const obstacle_detection_2019::synchronizedImage::ConstPtr& imageMsg){
    
    rgbImage = imageMsg->rgb;
    depthImage = imageMsg->depth;
}
//publish
void managementClass::publishRgbCamData(){//データ送信
    pubRgb.publish(rgbImage);
}
void managementClass::publishDepthCamData(){//データ送信
    pubDepth.publish(depthImage);
}
