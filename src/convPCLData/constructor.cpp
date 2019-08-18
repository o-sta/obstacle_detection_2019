#include<obstacle_detection_2019/convPCLData.h>

convPCLDataClass::convPCLDataClass()
	:sensorData_subscribed_flag(false)
{
    //subscriber
	nhSub.setCallbackQueue(&queue);
	sub=nhSub.subscribe("zed/point_cloud/cloud_registered",1,&convPCLDataClass::sensor_callback,this);
	//publisher
    pubRgb= nhPub1.advertise<sensor_msgs::Image>("converted_rgbImage", 1);
	//マスク画像
    pubDepth= nhPub2.advertise<sensor_msgs::Image>("converted_depthImage", 1);

}
convPCLDataClass::~convPCLDataClass(){

}

