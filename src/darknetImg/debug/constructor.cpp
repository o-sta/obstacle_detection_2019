#include <obstacle_detection_2019/darknetImgDebug.h>

darknetImgDebug::darknetImgDebug()
:mapImageCB(new cv_bridge::CvImage)
{
    ROS_INFO_STREAM("debug constructer");
    setCallback();
    setColorMap(colorMap);
    //setMapImageConfig(); //■error
    num_temp.resize(numberOfCells);
}

darknetImgDebug::~darknetImgDebug(){

}