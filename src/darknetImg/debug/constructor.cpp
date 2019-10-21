#include <obstacle_detection_2019/darknetImgDebug.h>

darknetImgDebug::darknetImgDebug()
:mapImageCB(new cv_bridge::CvImage)
{
    setParam();
    bbImage_pub = nhPub.advertise<sensor_msgs::Image>(topic_bbImage, 1);
    bbMaskImage_pub = nhPub.advertise<sensor_msgs::Image>(topic_bbMaskImage, 1);
    ROS_INFO_STREAM("debug constructer");
    setCallback();
    setColorMap(colorMap);
    //setMapImageConfig(); //■error
    num_temp.resize(numberOfCells);
}

darknetImgDebug::~darknetImgDebug(){
    
}