#include<obstacle_detection_2019/measurementVelocity.h>

measurementVelocity::measurementVelocity()
{

	//subscriber
	nhSub1.setCallbackQueue(&queue1);
	subCluster=nhSub1.subscribe("/classificationData",1,&measurementVelocity::cluster_callback,this);
	nhSub2.setCallbackQueue(&queue2);
	subMatch=nhSub2.subscribe("/imageMatchingData",1,&measurementVelocity::matching_callback,this);
	//publisher
    pub= nhPub.advertise<obstacle_detection_2019::ClassificationVelocityData>("measurementVelocityCluster", 1);

	//初回処理防止用
	prvClstr.header.seq = 0;

	//デバッグ用
	pubDeb= nhDeb.advertise<sensor_msgs::PointCloud2>("debugMeasuredVelocity", 1);

}
measurementVelocity::~measurementVelocity(){

}