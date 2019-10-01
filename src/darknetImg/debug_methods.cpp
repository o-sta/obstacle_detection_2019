#include <obstacle_detection_2019/darknetImgDebug.h>

darknetImgDebug::darknetImgDebug()
:mapImageCB(new cv_bridge::CvImage)
{
    setMapImageConfig();
}

darknetImgDebug::~darknetImgDebug(){

}

void darknetImgDebug::setColorMap(std::vector<int>& colorMap){
    nhSub.param("colorMap/data", colorMap, colorMap);
    colorMap.resize(colorMap.size() - (colorMap.size() % 3)); //要素数が3の倍数(RGB)になるようにリサイズ
}

void darknetImgDebug::setMapImageConfig(){
    mapImageRows = mapRows * (cellSideLength + cellMargin) + cellMargin; //最後のMarginは端の余白分
    mapImageCols = mapCols * (cellSideLength + cellMargin) + cellMargin; //最後のMarginは端の余白分
    mapImageCB->image = cv::Mat(mapImageRows, mapImageCols, CV_8UC3);
}


void darknetImgDebug::drawClusterCells(obstacle_detection_2019::ClassificationElement& cluster, int colorIndex){
    int mapRow, mapCol, mapImageRow_start, mapImageCol_start;
    cv::Scalar drawColor(colorMap[colorIndex*3], colorMap[colorIndex*3+1], colorMap[colorIndex*3+2]);
    for(auto c_i : cluster.index){ //クラスタに属しているセルで走査 n 
        //インデックスからrowとcol算出
        mapRow = c_i.data / mapRows;
        mapCol = c_i.data % mapCols;
        //セル描画
        mapImageRow_start = mapRow * (cellSideLength + cellMargin) + cellMargin; //描画開始位置設定 mapImageRow
        mapImageCol_start = mapCol * (cellSideLength + cellMargin) + cellMargin; //描画開始位置設定 mapImageCol
        cv::rectangle(mapImageCB->image, cv::Rect(mapImageRow_start, mapImageCol_start, cellSideLength, cellSideLength), drawColor, -1, CV_AA);
    }
}

void darknetImgDebug::cluster2Image(obstacle_detection_2019::ClassificationData& clusterData, cv::Mat& image){
    //クラスタに相当するセルを画像で表示する
    cv::rectangle(image, cv::Rect(0, 0, image.rows, image.cols), cv::Scalar(255,255,255), -1, CV_AA); //色初期化
    int colorIndex = 0; //色番号
    for (auto cluster : clusterData.data){
        drawClusterCells(cluster, colorIndex++);
    }
}


void darknetImgDebug::cluster2PointCloud(){
    //人の位置を３次元の点群で表示する
    
}

void darknetImgDebug::gridmap2Image(obstacle_detection_2019::ClassificationData& clusterData, cv::Mat& image){
    //セルに含まれる点の数を画像で表示するプログラム。閾値を用意し、それ以上を赤、それ以下を黒で表示するプログラムの開発を行う。
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