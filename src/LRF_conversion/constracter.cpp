#include<obstacle_detection_2019/convCamData.h>

convLRFDataClass::convLRFDataClass()
{

	//subscriber
	nhSub.setCallbackQueue(&queue);
	sub=nhSub.subscribe("/scan",1,&convLRFDataClass::sensor_callback,this);
	//publisher
    pub= nhPub.advertise<obstacle_detection::sensorMapData>("laserMapData", 1);

    //localMapParameter
    //後でlaunchから読み込みように変更
	mapW=8;//width[m]
	mapH=8;//height[m]
	mapR=0.05;//resolution[m]
	mapWi=(int)(mapW/mapR);//[pixel]
	mapHi=(int)(mapH/mapR);//[pixel]
	sensorHigh=0.7;//センサーの高さ
	
}
convLRFDataClass::~convLRFDataClass(){
	
}