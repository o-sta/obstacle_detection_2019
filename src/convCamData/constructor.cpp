#include<obstacle_detection_2019/convCamData.h>

convCamDataClass::convCamDataClass()
	:cloud(new pcl::PointCloud<pcl::PointXYZ>),
	inliers (new pcl::PointIndices),
	coefficients(new pcl::ModelCoefficients)
{

	//subscriber
	nhSub.setCallbackQueue(&queue);
	sub=nhSub.subscribe("/zed/depth/depth_registered",1,&convCamDataClass::sensor_callback,this);
	//publisher
    pubConv= nhPub1.advertise<obstacle_detection_2019::SensorMapData>("cameraMapData", 1);
	//マスク画像
    pubMask= nhPub2.advertise<obstacle_detection_2019::MaskImageData>("maskImageData", 1);

    //床面抽出パラメータ
	seg.setOptimizeCoefficients (true);
	//seg.setModelType (pcl::SACMODEL_PLANE);//全平面抽出
	seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);//ある軸に垂直な平面を抽出
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (500);//RANSACの繰り返し回数
	seg.setDistanceThreshold (0.10);//モデルとどのくらい離れていてもいいか(モデルの評価に使用)
	seg.setAxis(Eigen::Vector3f (0.0,0.0,1.0));//法線ベクトル
	seg.setEpsAngle(15.0f * (M_PI/180.0f));//許容出来る平面
	//床面除去パラメータ
	f=350.505;
	y_th=0.3;
	ground_th=0.2;
	cam_y=0.4125;
	height_th=1.0;
    //localMapParameter
    //後でlaunchから読み込みように変更
	mapW=8;//width[m]
	mapH=8;//height[m]
	mapR=0.05;//resolution[m]
	mapWi=(int)(mapW/mapR);//[pixel]
	mapHi=(int)(mapH/mapR);//[pixel]
	
}
convCamDataClass::~convCamDataClass(){

}