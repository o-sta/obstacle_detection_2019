#include<obstacle_detection_2019/velocityEstimation.h>

estimationClass::estimationClass()
{
	//subscriber
	// nhSub1.setCallbackQueue(&queue1);
	sub=nhSub1.subscribe("measurementVelocityCluster",1,&estimationClass::cluster_callback,this);
	//publisher
    pub= nhPub.advertise<obstacle_detection_2019::ClassificationVelocityData>("classificationData", 1);

	//launchファイルからパラメータの読み込み
	setLaunchParam();
	//デバッグ用
	//publisher
    // pubDeb= nhDeb.advertise<sensor_msgs::Image>("windowImage", 1);
    // pubDebPcl= nhDebPcl.advertise<sensor_msgs::PointCloud2>("visualizedCluster", 1);

	//rqt_reconfigure
	f = boost::bind(&estimationClass::configCallback, this, _1, _2);
	server.setCallback(f);

	//--calman filter parameter dynamicReconfigureに追加予定
	//resize
	sig_ut = Eigen::MatrixXd::Zero(2,2);
	del_t = Eigen::MatrixXd::Zero(4,4);
	sig_x0 = Eigen::MatrixXd::Zero(4,4);
	sig_wk = Eigen::MatrixXd::Zero(4,4);
	I = Eigen::MatrixXd::Identity(4,4);
	//initialize
	//delta Q
	//--
	del_t(0,0)=0.05*0.05;//0.01;//x
	del_t(1,1)=0.05*0.05;//0.01;//z
	del_t(2,2)=0.02;//0.09;//0.04;//0.25;//vx
	del_t(3,3)=0.02;//0.09;//0.04;//0.25;//vz
	//--
	del_t(0,2)=0;//del_t(0,0)*del_t(2,2)/2;
	del_t(1,3)=0;//del_t(1,1)*del_t(3,3)/2;
	del_t(2,0)=0;//del_t(0,2);//del_t(0,0)/(dt*dt);
	del_t(3,1)=0;//del_t(1,3);//del_t(1,1)/(dt*dt);
	//sigma
	//--
	sig_x0(0,0)=0.01;//0.04;//x
	sig_x0(1,1)=0.01;//0.04;//z
	sig_x0(2,2)=0.09;//vx
	sig_x0(3,3)=0.09;//vz
	//--
	sig_x0(0,2)=0;//sig_x0(2,2)*(0.2*0.2);
	sig_x0(2,0)=0;//sig_x0(0,2);//sig_x0(0,0)/(0.2*0.2);
	sig_x0(1,3)=0;//sig_x0(3,3)*(0.2*0.2);
	sig_x0(3,1)=0;//sig_x0(1,3);//sig_x0(1,1)/(0.2*0.2);
	//sigma
	//--
	sig_wk(0,0)=0.05*0.05;//x
	sig_wk(1,1)=0.05*0.05;//z
	sig_wk(2,2)=0.02;//0.01;//vx
	sig_wk(3,3)=0.02;//0.01;//vz
	//--
	sig_wk(0,2)=0;//sig_wk(0,0)*sig_wk(2,2)/2;
	sig_wk(1,3)=0;//sig_wk(3,3)*ig_wk(1,1)/2;
	sig_wk(2,0)=0;//sig_wk(0,2);//sig_wk(0,0)/(dt*dt);
	sig_wk(3,1)=0;//sig_wk(1,3);//sig_wk(1,1)/(dt*dt);


}
estimationClass::~estimationClass(){
}