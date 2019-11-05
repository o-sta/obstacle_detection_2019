#include <obstacle_detection_2019/darknetImgDebug.h>

darknetImgDebug::darknetImgDebug()
:mapImageCB(new cv_bridge::CvImage),
depth_points(new pcl::PointCloud<pcl::PointXYZRGB>)
{
    setParam();
    bbImage_pub = nhPub.advertise<sensor_msgs::Image>(topic_bbImage, 1);
    bbMaskImage_pub = nhPub.advertise<sensor_msgs::Image>(topic_bbMaskImage, 1);
    pcl_pub = nhPub.advertise<sensor_msgs::PointCloud2>(topic_clusterPCL, 1);
    pickUpGroundPointCandidates_pub = nhPub.advertise<sensor_msgs::PointCloud2>(topic_pickUpGroundPointCandidates, 1);
    depth2points_pub = nhPub.advertise<sensor_msgs::PointCloud2>(topic_depth2points, 1);
    estimateGroundCoefficients_pub = nhPub.advertise<sensor_msgs::PointCloud2>(topic_estimateGroundCoefficients, 1);
    removeGroundPoints_pub = nhPub.advertise<sensor_msgs::PointCloud2>(topic_removeGroundPoints, 1);
    trimPoints_pub = nhPub.advertise<sensor_msgs::Image>(topic_trimPoints, 1);
    gridMapPCL_pub = nhPub.advertise<sensor_msgs::PointCloud2>(topic_gridMapPCL, 1);
    ROS_INFO_STREAM("debug constructer");
    setCallback();
    setColorMap("colorMap/data", colorMap);
    nhPub.param("colorMap/grad_jet", colorMapGrad, colorMapGrad);
    colorMapGrad.resize(colorMapGrad.size() - (colorMapGrad.size() % 3)); //要素数が3の倍数(RGB)になるようにリサイズ
    num_temp.resize(numberOfCells);

}

darknetImgDebug::~darknetImgDebug(){
    
}