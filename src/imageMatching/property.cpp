#include<obstacle_detection_2019/imageMatching.h>

void imageMatchingClass::setLaunchParam(){
    
    ros::NodeHandle n("~");
    //
    n.getParam("imageMatching/debugType",debugType);
    //マップパラメータ
    n.getParam("localMap/width/float",mapW);
    n.getParam("localMap/height/float",mapH);
    n.getParam("localMap/resolution",mapR);
	mapWi=(int)(mapW/mapR);//[pixel]
	mapHi=(int)(mapH/mapR);//[pixel]
    //追跡閾値
    n.getParam("imageMatching/trackThreshold", trackThreshold);
}