#include<obstacle_detection_2019/imageMatching.h>

void imageMatchingClass::setLaunchParam(){
    
    ros::NodeHandle n("~");
    //
    n.getParam("imageMatching/debugType",debugType);
}