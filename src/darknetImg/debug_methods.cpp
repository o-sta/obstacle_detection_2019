#include <obstacle_detection_2019/darknetImgDebug.h>

darknetImgDebug::darknetImgDebug()
:mapImageCB(new cv_bridge::CvImage),
colorMap(1, 256, CV_8UC3)
{
    setMapImageConfig();
}

darknetImgDebug::~darknetImgDebug(){
    
}

void darknetImgDebug::setColorMap(){
    std::vector<int> data(3, 0);
    nhSub.param("colorMap/data", data, data);
    data.resize(data.size() - (data.size() % 3)); //要素数が3の倍数(RGB)になるようにリサイズ
    colorMap = cv::Mat(data, CV_8UC3);
}

void darknetImgDebug::setMapImageConfig(){
    mapImageRows = mapRows * (cellSideLength + cellMargin) + cellMargin; //最後のMarginは端の余白分
    mapImageCols = mapCols * (cellSideLength + cellMargin) + cellMargin; //最後のMarginは端の余白分
    mapImageCB->image = cv::Mat(mapImageRows, mapImageCols, CV_8UC3);
}


void darknetImgDebug::drawClusterCells(obstacle_detection_2019::ClassificationElement& cluster, int colorMode){
    int mapRow, mapCol, mapImageRow_start, mapImageCol_start;
    colorMode %= colorMap.cols;
    for(auto c_i : cluster.index){ //クラスタに属しているセルで走査 n 
        //インデックスからrowとcol算出
        mapRow = c_i.data / mapRows;
        mapCol = c_i.data % mapCols;
        //セル描画
        mapImageRow_start = mapRow * (cellSideLength + cellMargin) + cellMargin; //描画開始位置設定 mapImageRow
        mapImageCol_start = mapCol * (cellSideLength + cellMargin) + cellMargin; //描画開始位置設定 mapImageCol
        cv::rectangle(mapImageCB->image, cv::Rect(mapImageCol_start, mapImageRow_start, cellSideLength, cellSideLength), colorMap.at<cv::Scalar>[colorMode], 1, CV_AA);
    }
}

void darknetImgDebug::cluster2Image(){
    int count = 0;
    for (auto cluster : cd.data){
        drawClusterCells(cluster, count);
        ++count;
    }
}



// void darknetImg::sensor_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bb, const sensor_msgs::Image::ConstPtr& image)
// {
//     // debug移行予定
//     ROS_INFO_STREAM("callback : " << "[bb : " << bb->header.stamp.toSec() << " ]," 
//                                   << "[img : " << image->header.stamp.toSec() << " ],"
//                                   << "[diff : " << (bb->header.stamp - image->header.stamp).toSec() << "]");
//     // image から cvbridgeImage に変換する
//     try{
//         bridgeImage=cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);
//     }
//     catch(cv_bridge::Exception& e) {//エラー処理
//         ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.",image->encoding.c_str());
//         return;
//     }
//     // bounding_boxedに書かれた枠の描画
//     auto iter = bb->bounding_boxes.begin();
//     for (; iter != bb->bounding_boxes.end(); ++iter){
//         cv::rectangle(bridgeImage->image, cv::Point(iter->xmin, iter->ymin), cv::Point(iter->xmax, iter->ymax), cv::Scalar(0, 0, 200), 5, 8);
//     }
//     // パブリッシュ(debgu移行予定)
//     pub.publish(bridgeImage->toImageMsg());
// }