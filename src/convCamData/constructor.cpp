﻿#include<obstacle_detection_2019/convCamData.h>

convCamDataClass::convCamDataClass()
	:cloud(new pcl::PointCloud<pcl::PointXYZ>),
	inliers (new pcl::PointIndices),
	coefficients(new pcl::ModelCoefficients),
	f(711.010),mapW(8.0),mapH(8.0),mapR(0.05),
	ransacNum(500),distanceThreshold(0.1),epsAngle(15.0),
	groundCandidateY(0.3),camHeight(0.4125),ground_th(0.2),height_th(1.0)
{

	//subscriber
	nhSub.setCallbackQueue(&queue);
	sub=nhSub.subscribe("/zed/depth/depth_registered",1,&convCamDataClass::sensor_callback,this);
	//publisher
    pubConv= nhPub1.advertise<obstacle_detection_2019::SensorMapData>("cameraMapData", 1);
	//マスク画像
    pubMask= nhPub2.advertise<obstacle_detection_2019::MaskImageData>("maskImageData", 1);
	// lanchファイルの読み込み
	setLaunchParam();
    //床面抽出パラメータ
	seg.setOptimizeCoefficients (true);
	//seg.setModelType (pcl::SACMODEL_PLANE);//全平面抽出
	seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);//ある軸に垂直な平面を抽出
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (ransacNum);//RANSACの繰り返し回数
	seg.setDistanceThreshold (distanceThreshold);//モデルとどのくらい離れていてもいいか(モデルの評価に使用)
	seg.setAxis(Eigen::Vector3f (0.0,0.0,1.0));//法線ベクトル
	seg.setEpsAngle(epsAngle * (M_PI/180.0f));//許容出来る平面
	//床面除去パラメータ
	// f=350.505;
	// groundCandidateY=0.3;
	// ground_th=0.2;
	// camHeight=0.4125;
	// height_th=1.0;
	//
	sensorData_subscribed_flag = false; //sta
	
	//rqt_reconfigure
	fc = boost::bind(&convCamDataClass::configCallback, this, _1, _2);
	server.setCallback(fc);
}
convCamDataClass::~convCamDataClass(){

}