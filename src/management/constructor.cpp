#include<obstacle_detection_2019/management.h>

managementClass::managementClass(){
    //subscriber
	nhSub.setCallbackQueue(&queue);
	sub=nhSub.subscribe("syncronized_image",1,&managementClass::callback,this);
	//publisher
    //rgb
    pubRgb= nhPub1.advertise<sensor_msgs::Image>("converted_rgbImage", 1);
	//depth
    pubDepth= nhPub2.advertise<sensor_msgs::Image>("converted_depthImage", 1);

}
managementClass::~managementClass(){

}