#include<obstacle_detection_2019/measurementVelocity.h>

measurementVelocity::measurementVelocity()
{

	//subscriber
	nhSub1.setCallbackQueue(&queue1);
	subCam=nhSub1.subscribe("/",1,&measurementVelocity::cluster_callback,this);
	nhSub2.setCallbackQueue(&queue2);
	subLRF=nhSub2.subscribe("/",1,&measurementVelocity::matching_callback,this);
	//publisher
    pub= nhPub1.advertise<obstacle_detection_2019::MessageData>("publishdata", 1);

	//初回処理防止用
	prvClstr.header.seq = 0;

	
}
measurementVelocity::~measurementVelocity(){

}