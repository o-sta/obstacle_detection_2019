#include<obstacle_detection_2019/synchroImage.h>

void syncroImageClass::setLaunchParam(){
    
    ros::NodeHandle n("~");
    n.getParam("debugType",debugType);
}

