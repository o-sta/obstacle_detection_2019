#include<obstacle_detection_2019/convCamData.h>

void convCamDataClass::setLaunchParam(){
    
    ros::NodeHandle n("~");
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