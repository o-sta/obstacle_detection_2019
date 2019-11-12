#include <obstacle_detection_2019/darknetImgDebug.h>

void darknetImgDebug::setParam(){
    darknetImg::setParam();
    nhPub.param<std::string>("topic/publisher/bbImage", topic_bbImage, "bbImage");
    nhPub.param<std::string>("topic/publisher/bbMaskImage", topic_bbMaskImage, "bbMaskImage");
    nhPub.param<std::string>("topic/publisher/clusterImage", topic_clusterImage, "clusterImage");
    nhPub.param<std::string>("topic/publisher/clusterPCL", topic_clusterPCL, "clusterPCL");
    nhPub.param<std::string>("topic/publisher/gridMapImage", topic_gridMapImage, "gridMapImage");
    nhPub.param<std::string>("topic/publisher/depth2points", topic_depth2points, "depth2points");
    nhPub.param<std::string>("topic/publisher/pickUpGroundPointCandidates", topic_pickUpGroundPointCandidates, "pickUpGroundPointCandidates");
    nhPub.param<std::string>("topic/publisher/estimateGroundCoefficients", topic_estimateGroundCoefficients, "estimateGroundCoefficients");
    nhPub.param<std::string>("topic/publisher/removeGroundPoints", topic_removeGroundPoints, "removeGroundPoints");
    nhPub.param<std::string>("topic/publisher/trimPoints", topic_trimPoints, "trimPoints");
    nhPub.param<std::string>("topic/publisher/gridMapPCL", topic_gridMapPCL, "gridMapPCL");
    ROS_INFO_STREAM("setparam debug mode");
}

void darknetImgDebug::setCallback(){
    ROS_INFO_STREAM("debug setCallback");
    sync.init();
    sync.registerCallback(boost::bind(&darknetImgDebug::debug_callback, this, _1, _2));
    // fc = boost::bind(&darknetImg::configCallback, this, _1, _2);
	// server.setCallback(fc);
}

void darknetImgDebug::setColorMap(std::string paramName, std::vector<int>& colorMap){
    nhPub.param(paramName, colorMap, colorMap);
    colorMap.resize(colorMap.size() - (colorMap.size() % 3)); //要素数が3の倍数(RGB)になるようにリサイズ
}

void darknetImgDebug::setMapImageConfig(){
    mapImageRows = mapRows * (cellSideLength + cellMargin) + cellMargin; //最後のMarginは端の余白分
    mapImageCols = mapCols * (cellSideLength + cellMargin) + cellMargin; //最後のMarginは端の余白分
    mapImageCB->image = cv::Mat(mapImageRows, mapImageCols, CV_8UC3);
}

cv::Scalar darknetImgDebug::getColorFromColorMap(int colorIndex){
    cv::Scalar drawColor(colorMap[(colorIndex*3)%colorMap.size()], colorMap[(colorIndex*3+1)%colorMap.size()], colorMap[(colorIndex*3+2)%colorMap.size()]);
    return drawColor;
}
