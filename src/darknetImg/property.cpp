#include <obstacle_detection_2019/darknetImg.h>

void darknetImg::setParam(){
    ROS_INFO_STREAM("debug setParam");
    nhPub.param<std::string>("topic/subscriber/boundingBoxes", topic_bb, "darknet_ros/bounding_boxes");
    nhPub.param<std::string>("topic/subscriber/depthImage", topic_depthImage, "left/image_rect_color");
    nhPub.param<float>("camera/focus", f, f);
    nhPub.param<float>("localMap/width", mapWidth, 8.0);
    nhPub.param<float>("localMap/height", mapHeight, 8.0);
    nhPub.param<float>("localMap/resolution", mapResolution, 0.05);
    nhPub.param<float>("groundEstimate/cameraHeight", camHeight, 0.3);
    nhPub.param<float>("groundEstimate/candidateY", groundCandidateY, 0.5);
    nhPub.param<int>("groundEstimate/ransac/num", ransacNum, 10);
    nhPub.param<float>("groundEstimate/ransac/distanceThreshold", distanceThreshold, 0.5);
    nhPub.param<float>("groundEstimate/ransac/epsAngle", epsAngle, 15.0);
    nhPub.param<int>("window/minPts", minPts, 1600);
    nhPub.param<int>("window/rangeCell", windowRangeCell, 2);
    ROS_INFO_STREAM("focus value" << f);
}

void darknetImg::configCallback(obstacle_detection_2019::darknetImgConfig &config, uint32_t level){
    ROS_INFO_STREAM("Reconfigure Request:"
                    << " " << config.cameraHeight
                    << " " << config.groundThreshold
                    << " " << config.windowMinPts
                    << " " << config.windowRangeCell
                    << " " << config.ransacNum
                    << " " << config.ransacDistanceThreshold
                    << " " << config.ransacEpsAngle
                    << " " << config.estimateCandidateY
                    );
    //ransac パラメータ
    ransacNum = config.ransacNum;
    distanceThreshold = config.ransacDistanceThreshold;
    epsAngle = config.ransacEpsAngle;
    //ground パラメータ
    groundCandidateY = config.estimateCandidateY;
    camHeight = config.cameraHeight;
    ground_th = config.groundThreshold;
    //window パラメータ
    minPts = config.windowMinPts;
    windowRangeCell = config.windowRangeCell;   
    createWindow();
}

void darknetImg::createWindow(){
    cellsInWindow.resize(mapRows*mapCols);
    int count = 0;
    for(int row=-1; row<2; row++){
            cellsInWindow[count++] = row;
    }
    cellsInWindow.resize(count);
}

void darknetImg::setCallback(){
    sync.registerCallback(boost::bind(&darknetImg::sensor_callback, this, _1, _2));
    fc = boost::bind(&darknetImg::configCallback, this, _1, _2);
	server.setCallback(fc);
}

