#include <obstacle_detection_2019/darknetImg.h>

darknetImg::darknetImg(/* args */)
:
nhPub("~"),
// bb_sub(nhSub, "/darknet_ros/bounding_boxes", 1), 
// image_sub(nhSub, "/zed/zed_node/left/image_rect_color", 1), 
bb_sub(), 
image_sub(), 
sync(MySyncPolicy(10),bb_sub, image_sub),
is_size_initialized(false),
mask(1,1,CV_8UC1), 
ground_points(new pcl::PointCloud<pcl::PointXYZ>),
inliers (new pcl::PointIndices),
coefficients(new pcl::ModelCoefficients)
{
    setParam(); //パラメータのセットアップ
    pub = nhPub.advertise<sensor_msgs::Image>("/dphog/boximage", 1);
    ROS_INFO_STREAM("debug setParam");
    bb_sub.subscribe(nhSub, topic_bb, 1);
    image_sub.subscribe(nhSub, topic_depthImage, 1);
    mapRows = (int)(mapHeight/mapResolution);
    mapCols = (int)(mapWidth/mapResolution);
    numberOfCells = mapRows*mapCols;
    cellsInWindow.resize(mapRows*mapCols);
    int count = 0;
    for(int row=-1; row<2; row++){
            cellsInWindow[count++] = row;
    }
    cellsInWindow.resize(count);
    //RANSACパラメータ設定
    seg.setOptimizeCoefficients (true);
	//seg.setModelType (pcl::SACMODEL_PLANE);//全平面抽出
	seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);//ある軸に垂直な平面を抽出
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (ransacNum);//RANSACの繰り返し回数
	seg.setDistanceThreshold (distanceThreshold);//モデルとどのくらい離れていてもいいか(モデルの評価に使用)
	seg.setAxis(Eigen::Vector3f (0.0,0.0,1.0));//法線ベクトル
	seg.setEpsAngle(epsAngle * (M_PI/180.0f));//許容出来る平面
    ROS_INFO_STREAM("Started darknetImg");
    // setCallback();  //コールバック関数のセットアップ
}

darknetImg::~darknetImg(){}
