#include<obstacle_detection_2019/convCamData.h>

void convCamDataClass::setLaunchParam(){
    
    ros::NodeHandle n("~");
    n.getParam("groundEstimate/rqt_reconfigure",rqt_reconfigure);
    //カメラパラメータ
    n.getParam("camera/focus",f);
    //マップパラメータ
    n.getParam("localMap/width/float",mapW);
    n.getParam("localMap/height/float",mapH);
    n.getParam("localMap/resolution",mapR);
	mapWi=(int)(mapW/mapR);//[pixel]
	mapHi=(int)(mapH/mapR);//[pixel]
    //ransac パラメータ
    n.getParam("groundEstimate/ransac/num",ransacNum);
    n.getParam("groundEstimate/ransac/distanceThreshold",distanceThreshold);
    n.getParam("groundEstimate/ransac/epsAngle",epsAngle);
    // 床面パラメータ
    n.getParam("groundEstimate/candidateY",groundCandidateY);
    n.getParam("groundEstimate/cameraHeight",camHeight);
    n.getParam("groundEstimate/groundThreshold",ground_th);
    n.getParam("groundEstimate/heightThreshold",height_th);
}
void convCamDataClass::configCallback(obstacle_detection_2019::convCamDataConfig &config, uint32_t level) {
	ROS_INFO("Reconfigure Request: %d %f %f %f %f %f %f", 
		config.ransacNum, config.ransacDistanceThreshold,
		config.ransacEpsAngle, config.estimateCandidateY,
		config.estimateCameraHeight, config.estimateGroundThreshold,
		config.estimateHeightThreshold
		);
    //ransacパラメータ
    ransacNum = config.ransacNum;
    distanceThreshold = config.ransacDistanceThreshold;
    epsAngle = config.ransacEpsAngle;
    //ground パラメータ
    groundCandidateY = config.estimateCandidateY;
    camHeight = config.estimateCameraHeight;
    ground_th = config.estimateGroundThreshold;
    height_th = config.estimateHeightThreshold;
}