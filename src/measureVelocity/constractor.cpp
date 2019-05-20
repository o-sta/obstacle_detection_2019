#include<obstacle_detection_2019/convCamData.h>

classificationClass::classificationClass()
{

	//subscriber
	nhSub1.setCallbackQueue(&queue1);
	subCam=nhSub1.subscribe("/",1,&classificationClass::cluster_callback,this);
	nhSub2.setCallbackQueue(&queue2);
	subLRF=nhSub2.subscribe("/",1,&classificationClass::matching_callback,this);
	//publisher
    pub= nhPub1.advertise<obstacle_detection::messageData>("publishdata", 1);

	//初回処理防止用
	prvClstr.header.seq = 0;

	
}
classificationClass::~classificationClass(){

}