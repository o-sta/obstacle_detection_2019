#include<obstacle_detection_2019/classification.h>

void classificationClass::setWindowParam(){
    
    ros::NodeHandle n("~");
    n.getParam("windowDivisionDegree",winDivDeg);
    winDivNum = (int)( (maxCamDeg - minCamDeg) / winDivDeg )*2 + 1;
}