#include<obstacle_detection_2019/velocityEstimation.h>

void estimationClass::setLaunchParam(){
    
    ros::NodeHandle n("~");
    //カルマンフィルタパラメータ
	//--観測誤差共分散
    n.getParam("observationDelta/1/1",del_t(0,0));
    n.getParam("observationDelta/2/2",del_t(1,1));
    n.getParam("observationDelta/3/3",del_t(2,2));
    n.getParam("observationDelta/4/4",del_t(3,3));
    n.getParam("observationDelta/1/3",del_t(0,2));
    n.getParam("observationDelta/3/1",del_t(2,0));
    n.getParam("observationDelta/2/4",del_t(1,3));
    n.getParam("observationDelta/4/2",del_t(3,1));
	//--モデル誤差共分散
    n.getParam("predictionSigma/1/1",sig_wk(0,0));
    n.getParam("predictionSigma/2/2",sig_wk(1,1));
    n.getParam("predictionSigma/3/3",sig_wk(2,2));
    n.getParam("predictionSigma/4/4",sig_wk(3,3));
    n.getParam("predictionSigma/1/3",sig_wk(0,2));
    n.getParam("predictionSigma/3/1",sig_wk(2,0));
    n.getParam("predictionSigma/2/4",sig_wk(1,3));
    n.getParam("predictionSigma/4/2",sig_wk(3,1));
	//--推定共分散の初期値
    n.getParam("estimationSigma/1/1",sig_x0(0,0));
    n.getParam("estimationSigma/2/2",sig_x0(1,1));
    n.getParam("estimationSigma/3/3",sig_x0(2,2));
    n.getParam("estimationSigma/4/4",sig_x0(3,3));
    n.getParam("estimationSigma/1/3",sig_x0(0,2));
    n.getParam("estimationSigma/3/1",sig_x0(2,0));
    n.getParam("estimationSigma/2/4",sig_x0(1,3));
    n.getParam("estimationSigma/4/2",sig_x0(3,1));
}