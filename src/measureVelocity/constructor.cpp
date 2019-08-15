#include<obstacle_detection_2019/measurementVelocity.h>

measurementVelocity::measurementVelocity()
	 :debugType(1), timeRange(10), timeInteval(1), 
	 weightImage(1), weightSize(1), weightGravity(1),
	 trackThreshold(1)
{

	//subscriber
	// nhSub1.setCallbackQueue(&queue1);
	subCluster=nhSub1.subscribe("classificationData",1,&measurementVelocity::cluster_callback,this);
	// nhSub2.setCallbackQueue(&queue2);
	subMatch=nhSub2.subscribe("imageMatchingData",1,&measurementVelocity::matching_callback,this);
	//publisher
    pub= nhPub.advertise<obstacle_detection_2019::ClassificationVelocityData>("measurementVelocityCluster", 1);

	//初回処理防止用
	prvClstr.header.seq = 0;

	//デバッグ用
	pubDeb= nhDeb.advertise<sensor_msgs::PointCloud2>("debugMeasuredVelocity", 1);

	//launchファイルの読み込み
	setLaunchParam();
	
	//rqt_reconfigure
	f = boost::bind(&measurementVelocity::configCallback, this, _1, _2);
	server.setCallback(f);
}
measurementVelocity::~measurementVelocity(){

}