#include <obstacle_detection_2019/darknetImgDebug.h>

darknetImgDebug::darknetImgDebug()
:mapImageCB(new cv_bridge::CvImage)
{
    setMapImageConfig();
    num_temp.resize(numberOfCells);
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


void darknetImgDebug::drawClusterCells(obstacle_detection_2019::ClassificationElement& cluster, int colorIndex, cv::Mat& image){
    int mapRow, mapCol, mapImageRow_start, mapImageCol_start;
    cv::Scalar drawColor(colorMap[(colorIndex*3)%colorMap.size()], colorMap[(colorIndex*3+1)%colorMap.size()], colorMap[(colorIndex*3+2)%colorMap.size()]);
    for(auto c_i : cluster.index){ //クラスタに属しているセルで走査 n 
        //インデックスからrowとcol算出
        mapRow = c_i.data / mapRows;
        mapCol = c_i.data % mapCols;
        //セル描画
        mapImageRow_start = mapRow * (cellSideLength + cellMargin) + cellMargin; //描画開始位置設定 mapImageRow
        mapImageCol_start = mapCol * (cellSideLength + cellMargin) + cellMargin; //描画開始位置設定 mapImageCol
        cv::rectangle(image, cv::Rect(mapImageRow_start, mapImageCol_start, cellSideLength, cellSideLength), drawColor, -1, CV_AA);
    }
}

void darknetImgDebug::cluster2Image(obstacle_detection_2019::ClassificationData& clusterData, cv::Mat& image){
    //クラスタに相当するセルを画像で表示する
    cv::rectangle(image, cv::Rect(0, 0, image.rows, image.cols), cv::Scalar(255,255,255), -1, CV_AA); //色初期化
    int colorIndex = 0; //色番号
    for (auto cluster : clusterData.data){
        drawClusterCells(cluster, colorIndex++, image);
    }
}


void darknetImgDebug::cluster2PointCloud(){
    //人の位置を３次元の点群で表示する
    
}

void darknetImgDebug::gridmap2Image(obstacle_detection_2019::SensorMapDataMultiLayer& smdml,int pt_num_threthold ,cv::Mat& image){
    //セルに含まれる点の数を画像で表示するプログラム。閾値を用意し、それ以上を赤、それ以下を黒で表示するプログラムの開発を行う。
    //各レイヤーのセルに含まれている点の数を数える
    int count = 0;
    int max_pt_num = 0; //セルに含まれていた最大点
    std::fill(num_temp.begin(), num_temp.end(), 0); //テンプレートの初期化
    for(auto layer : smdml.layer){//各レイヤーを走査
        count = 0;
        for(auto cell : layer.index){ //各レイヤーのセルを統合
            if(cell.data != -1){
                num_temp[count] += layer.size[count].data;
            }
            count++;
        }
        for(auto cell_pt_num : num_temp){ //最大点数のセルを探査する
            if(max_pt_num < cell_pt_num){
                max_pt_num = cell_pt_num;
            }
        }
    }
    // 描画
    int mapRow, mapCol, mapImageRow_start, mapImageCol_start, c_i;
    cv::Scalar red_color(0, 0, 255);
    cv::Scalar black_color(50,50,50);
    for(auto cell_pt_num : num_temp){ //クラスタに属しているセルで走査
        //インデックスからrowとcol算出
        mapRow = c_i / mapRows;
        mapCol = c_i % mapCols;
        //セル描画
        mapImageRow_start = mapRow * (cellSideLength + cellMargin) + cellMargin; //描画開始位置設定 mapImageRow
        mapImageCol_start = mapCol * (cellSideLength + cellMargin) + cellMargin; //描画開始位置設定 mapImageCol
        if(cell_pt_num < pt_num_threthold){
            cv::rectangle(image, cv::Rect(mapImageRow_start, mapImageCol_start, cellSideLength, cellSideLength), black_color, -1, CV_AA);
        }else{
            cv::rectangle(image, cv::Rect(mapImageRow_start, mapImageCol_start, cellSideLength, cellSideLength), red_color, -1, CV_AA);
        }
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