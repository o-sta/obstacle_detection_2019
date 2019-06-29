#include<obstacle_detection_2019/velocityEstimation.h>

//subscribe
void estimationClass::cluster_callback(const obstacle_detection_2019::ClassificationVelocityData::ConstPtr& msg)
{
    //データをコピー
	curClstr = *msg;
	//move manage method
	manage();
}
void estimationClass::configCallback(obstacle_detection_2019::estimationConfig &config, uint32_t level) {
	// ROS_INFO("Reconfigure Request: %d %f %f %d", 
	// 	config.windowDivisionDegree, config.windowHeight,
	// 	config.windowWidth,config.windowMinPts
	// 	// config.str_param.c_str(), 
	// 	// config.bool_param?"True":"False", 
	// 	// config.size
	// 	);
	//観測誤差共分散
	del_t(0,0) = config.observationDelta11;
	del_t(1,1) = config.observationDelta22;
	del_t(2,2) = config.observationDelta33;
	del_t(3,3) = config.observationDelta44;
	del_t(0,2) = config.observationDelta13;
	del_t(1,3) = config.observationDelta31;
	del_t(2,0) = config.observationDelta24;
	del_t(3,1) = config.observationDelta42;
	//モデル誤差共分散
	sig_wk(0,0) = config.predictionSigma11;
	sig_wk(1,1) = config.predictionSigma22;
	sig_wk(2,2) = config.predictionSigma33;
	sig_wk(3,3) = config.predictionSigma44;
	sig_wk(0,2) = config.predictionSigma13;
	sig_wk(1,3) = config.predictionSigma31;
	sig_wk(2,0) = config.predictionSigma24;
	sig_wk(3,1) = config.predictionSigma42;
	//推定共分散の初期値
	sig_x0(0,0) = config.estimationSigma11;
	sig_x0(1,1) = config.estimationSigma22;
	sig_x0(2,2) = config.estimationSigma33;
	sig_x0(3,3) = config.estimationSigma44;
	sig_x0(0,2) = config.estimationSigma13;
	sig_x0(1,3) = config.estimationSigma31;
	sig_x0(2,0) = config.estimationSigma24;
	sig_x0(3,1) = config.estimationSigma42;

}
void estimationClass::manage(){
	if(!isPreCluster()){
		ROS_INFO("First process");
		renewMessages();
		return ;
	}
	kalmanFilter();
	//
	publishData();
	renewMessages();
}
bool estimationClass::isPreCluster(){
	if(preClstr.header.seq > 0){
		return true;
	}
	return false;
}
void estimationClass::kalmanFilter(){//
	//カルマンフィルタ後クラスタデータの初期化
	filtedClstr = curClstr;
	//define
	Eigen::MatrixXd K_t(4,4);
	Eigen::MatrixXd xt_t(4,1);
	Eigen::MatrixXd z_t(4,1);
	Eigen::MatrixXd F_t = Eigen::MatrixXd::Zero(4,4);
	Eigen::MatrixXd B_t = Eigen::MatrixXd::Zero(2,2);
	Eigen::MatrixXd sig_xt(4,4);
	Eigen::MatrixXd u_t(4,1);
	//resize
	xh_t_1.resize(xh_t.size());
	sig_xh_t_1.resize(sig_xh_t.size());
	xh_t_1.resize(curClstr.data.size());
	sig_xh_t_1.resize(curClstr.data.size());

	float dt = curClstr.dt;
	//initialize
	//F
	F_t(0,0)=1;
	F_t(1,1)=1;
	F_t(2,2)=1;
	F_t(3,3)=1;
	F_t(0,2)=dt;
	F_t(1,3)=dt;
	//
	for(int i=0;i<curClstr.data.size();i++)
	{
		z_t(0,0)=curClstr.data[i].gc.x;
		z_t(1,0)=curClstr.data[i].gc.y;
		z_t(2,0)=curClstr.twist[i].linear.x;
		z_t(3,0)=curClstr.twist[i].linear.y;
		//can't do filter
		if(curClstr.trackingNum[i]<2)//1:no tracking,2:vel
		{
			// xh_t_1[i]=z_t;//観測データを推定結果としてそのまま出力
			// sig_xh_t_1[i]=sig_x0;//推定値の共分散は初期分散
			continue;
		}
		//推定位置速度、共分散を更新
		xh_t_1[i]=xh_t[ curClstr.match[i] ];
		sig_xh_t_1[i]=sig_xh_t[ curClstr.match[i] ];
	}
	//初期化
	xh_t.resize(curClstr.data.size());
	sig_xh_t.resize(curClstr.data.size());


	for(int i=0;i<curClstr.data.size();i++)
	{
		//set z
		z_t(0,0)=curClstr.data[i].gc.x;
		z_t(1,0)=curClstr.data[i].gc.y;
		z_t(2,0)=curClstr.twist[i].linear.x;
		z_t(3,0)=curClstr.twist[i].linear.y;
		//can't do filter
		if(curClstr.trackingNum[i] < 2)
		{
			xh_t[i]=z_t;//観測データを推定結果としてそのまま出力
			sig_xh_t[i]=sig_x0;//推定値の共分散は初期分散
			continue;
		}
		//filtering
		float dt = curClstr.dt;
		//set ut
		u_t(0,0)=preClstr.twist[ curClstr.match[i] ].linear.x;//vel[i].x;
		u_t(1,0)=preClstr.twist[ curClstr.match[i] ].linear.z;//vel[i].z;
		//
		//カルマンフィルタ更新ステップ
		xt_t = F_t*xh_t_1[i];// + B_t*u_t ;
		sig_xt = F_t * sig_xh_t_1[i] * F_t.transpose()+sig_wk;// + B_t * sig_ut * B_t.transpose() ;
		K_t = sig_xt*( (sig_xt+del_t).inverse() );
		xh_t[i] = xt_t + K_t*( z_t - xt_t );
		sig_xh_t[i] = (I - K_t)*sig_xt;
		//推定後データの格納
		filtedClstr.twist[i].linear.x = xh_t[i](2,0);
		filtedClstr.twist[i].linear.y = xh_t[i](3,0);
	}
}

void estimationClass::publishData(){//データ送信
    pub.publish(filtedClstr);
}
void estimationClass::renewMessages(){
	preClstr = curClstr;
}