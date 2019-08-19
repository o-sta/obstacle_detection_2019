#include<obstacle_detection_2019/recordExData.h>

recordExData::recordExData()
	:robotNum(2),distanceThreshold(1.0),rqt_reconfigure(true),outputCSV(false),
	userpath("/home/ros-robot/"),filepath("ex_data/test/"),filename("sample")
{
	//subscriber
	sub1=nhSub.subscribe("classificationDataEstimateVelocity",1,&recordExData::velCluster_callback,this);
	sub2=nhSub.subscribe("/robot1/odom",1,&recordExData::odom_callback,this);

	//publisher
    pub= nhPub.advertise<obstacle_detection_2019::recordData>("recordData", 1);

	//デバッグ用
	pubDebMarker= nhDeb.advertise<visualization_msgs::MarkerArray>("compVelocityMarker", 1);

	//launchファイルからパラメータの読み込み
	setLaunchParam();

	//ビーゴの座標系に固定
	turtle_odom.header = init_turtle_odom.header;
	measure_odom.header = init_turtle_odom.header;
	beego_odom.header = init_beego_odom.header;
	estimate_odom.header = init_beego_odom.header;
	//rqt_reconfigure
    if(rqt_reconfigure){
		f = boost::bind(&recordExData::configCallback, this, _1, _2);
		server.setCallback(f);
	}
	//csv データラベル
	if(outputCSV){
		setCSVDataLabel();
	}
}
recordExData::~recordExData(){
}