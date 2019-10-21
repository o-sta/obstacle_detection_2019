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

void darknetImgDebug::debug_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bb,const sensor_msgs::Image::ConstPtr& image){
    ROS_INFO_STREAM("debug callback : " << "[bb : " << bb->header.stamp.toSec() << " ]," 
                                  << "[img : " << image->header.stamp.toSec() << " ],"
                                  << "[diff : " << (bb->header.stamp - image->header.stamp).toSec() << "]");
    // image から cvbridgeImage に変換する
    try{
        bridgeImage=cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);
    }
    catch(cv_bridge::Exception& e) {//エラー処理
        ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.",image->encoding.c_str());
        return;
    }
    //mask2.resize(bridgeImage->image.rows);
    // mask2.resize(bridgeImage);
    //枠線描画用コード　debugの方に移動予定
    // bounding_boxedに書かれた枠の描画
    // auto iter = bb->bounding_boxes.begin();
    // for (; iter != bb->bounding_boxes.end(); ++iter){
    //     cv::rectangle(bridgeImage->image, cv::Point(iter->xmin, iter->ymin), cv::Point(iter->xmax, iter->ymax), cv::Scalar(0, 0, 200), 5, 8);
    // }
    // // パブリッシュ(debgu移行予定)
    // pub.publish(bridgeImage->toImageMsg());
    boundingBoxesMsg = *bb;
    if (imageRows != bridgeImage->image.rows || imageCols != bridgeImage->image.cols){
        imageRows = bridgeImage->image.rows;
        imageCols = bridgeImage->image.cols;
        setMaskSize();
    }
    int imageSize = imageRows * imageCols;
    int it = 0;
    auto *p = mask.ptr<char>(0);
    while(it < imageSize){
        *p = 0;
        ++p;
        ++it;
    }
    trimPoints_v2(boundingBoxesMsg);
    cv_bridge::CvImage cvMask;
    cvMask.header = bridgeImage->header;
    cvMask.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    cvMask.image = mask.clone();
    pub.publish(bridgeImage->toImageMsg());
    // pub.publish(cvMask.toImageMsg());
    // if (bb->bounding_boxes.size() > 0){
    //     ROS_INFO_STREAM("pickUpGroundPointCandidates");
    //     pickUpGroundPointCandidates();
    //     ROS_INFO_STREAM("estimateGroundCoefficients");
    //     estimateGroundCoefficients();
    //     ROS_INFO_STREAM("removeGroundPoints");
    //     removeGroundPoints();
    //     ROS_INFO_STREAM("trimPoints");
    //     trimPoints();
    //     ROS_INFO_STREAM("generateGridmap");
    //     generateGridmap();
    //     ROS_INFO_STREAM("dimensionalityReductionGridmap");
    //     dimensionalityReductionGridmap();
    //     ROS_INFO_STREAM("classifyPoints");
    //     classifyPoints();
    //     ROS_INFO_STREAM("estimatePersonPosition");
    //     estimatePersonPosition();
    //     ROS_INFO_STREAM("predictPersonPosition");
    //     predictPersonPosition();
    //     ROS_INFO_STREAM("iteration finished");
    // }else{
    //     ROS_INFO_STREAM("not person detection");
    // }
    // clearMsg(smdml);
    //ROS_INFO_STREAM("----------------------------------------");
}


void darknetImgDebug::setParam(){
    darknetImg::setParam();
    nhPub.param<std::string>("topic/publisher/image", topic_image, "image");
    nhPub.param<std::string>("topic/publisher/clusterImage", topic_clusterImage, "clusterImage");
    nhPub.param<std::string>("topic/publisher/clusterPCL", topic_clusterPCL, "clusterPCL");
    nhPub.param<std::string>("topic/publisher/gridMapImage", topic_gridMapImage, "gridMapImage");
    nhPub.param<std::string>("topic/publisher/mask", topic_mask, "mask");
}

void darknetImgDebug::setCallback(){
    ROS_INFO_STREAM("debug setCallback");
    sync.init();
    sync.registerCallback(boost::bind(&darknetImgDebug::debug_callback, this, _1, _2));
    // fc = boost::bind(&darknetImg::configCallback, this, _1, _2);
	// server.setCallback(fc);
}

void darknetImgDebug::setColorMap(std::vector<int>& colorMap){
    nhPub.param("colorMap/data", colorMap, colorMap);
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


void darknetImgDebug::cluster2PointCloud(obstacle_detection_2019::ClassificationData& clusterData, sensor_msgs::PointCloud2& pc_msg){
    //人の位置を３次元の点群で表示する
    pc_msg.header = clusterData.header;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr viewCloud(new  pcl::PointCloud<pcl::PointXYZRGB>);
    //初期化
    //データ数の計算
    int pointNum = 0;//総データ数
    for(auto element : clusterData.data){
        pointNum += (int)element.pt.size();
    }
    //表示高さ幅設定(点１つでは見えにくいため表示数を増やす)
    //表示範囲, 表示個数
    //z+zUnder <= z <= z+zUpper
    //(zUpper - zUnder)/zDelta
    float zUpper =0.2;
    float zUnder = -0.2;
    float zDelta =0.05;
    int zLoop = (int)((zUpper - zUnder)/zDelta) + 1;//ループ回数(z軸表示幅)
    //データ数再計算
    pointNum = pointNum * zLoop;
	viewCloud->points.clear();
	viewCloud->points.resize(pointNum);
    
    //要素追加用の仮変数
	pcl::PointXYZRGB cloudTemp;
    //追加済み要素数カウント
    int count = 0;
    //pointCloudデータ作成
    viewCloud->width = count;
    viewCloud->height = 1;
    int colorIndex = 0;
    //各クラスタごとの処理
    // for(int i = 0; i < cd.data.size(); i++){
    for(auto element : clusterData.data){
        //非表示処理
        //--データ数が閾値以下の時
        // if(cd.data[i].dens.data < 10){
        //     continue;
        // }
        //カラー設定
        cloudTemp.r=colorMap[(colorIndex*3)%colorMap.size()];
        cloudTemp.g=colorMap[(colorIndex*3+1)%colorMap.size()];
        cloudTemp.b=colorMap[(colorIndex*3+2)%colorMap.size()];
        colorIndex++;
        //各データごとの処理
        // for(int k = 0; k < element.pt.size(); k++){  
        for(auto pt_i : element.pt){
            cloudTemp.x = pt_i.y;//y軸              
            cloudTemp.y = pt_i.x;//逆向きのx軸
            //表示幅分点を追加
            for(int n=0; n <= (int)((zUpper - zUnder)/zDelta); n++){
                cloudTemp.z=pt_i.z + zUnder + n*zDelta;//z軸 + 表示範囲            
                //ポイントクラウドに追加
                viewCloud->points[count++] = cloudTemp;
            }
        }
    }
    viewCloud->width = count; //上のループの中に入れる？■
	viewCloud->points.resize(count);
	std::cout<<"viewCloud->points.size():"<<viewCloud->points.size()<<"\n";
    //データがないとき
	if(viewCloud->width <= 0)
	{
        ROS_INFO("No point cloud data!");
		return ;
	}
	
	pcl::toROSMsg (*viewCloud, pc_msg);
    //ここから先は別の関数で行う
	// pc_msg.header.frame_id="/zed_camera_center";
	// pubDebPcl.publish(viewMsgs);
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

void darknetImgDebug::addBBGroupRecursively(darknet_ros_msgs::BoundingBoxes& bbs, std::vector<bool>& checkFlag, int coreNumber, int groupNumber){
    int searchNumber = 0;
    std::stringstream ss;
    int text_row = 0;
    checkFlag[coreNumber] = true;
    for(searchNumber = 0; searchNumber < checkFlag.size(); ++searchNumber){
        if(!checkFlag[searchNumber]){
            switch (checkBoundingBoxesRelationship(bbs, coreNumber, searchNumber)) { //他のBBとの関連を調べる
                case darknetImg::Relationship::MIX: //一部含まれている場合は同groupであり、探査BBをcoreにして再度関連を調べる -> 探査済みにする
                    addBBGroupRecursively(bbs, checkFlag, searchNumber, groupNumber);
                    ss << "MIX " << coreNumber << "(" << bbs.bounding_boxes[coreNumber].xmin << "," << bbs.bounding_boxes[coreNumber].ymin << ")-" 
                                    << "(" << bbs.bounding_boxes[coreNumber].xmax << "," << bbs.bounding_boxes[coreNumber].ymax << ")" 
                                    << " : " << searchNumber << "(" << bbs.bounding_boxes[searchNumber].xmin << "," << bbs.bounding_boxes[searchNumber].ymin << ")-"
                                    << "(" << bbs.bounding_boxes[searchNumber].xmax << "," << bbs.bounding_boxes[searchNumber].ymax << ") in" 
                                    << groupNumber << " group";
                    break;
                case darknetImg::Relationship::NONE: //含まれていない場合は別groupであるため、何もしない
                    ss << "NONE " << coreNumber << "(" << bbs.bounding_boxes[coreNumber].xmin << "," << bbs.bounding_boxes[coreNumber].ymin << ")-" 
                                    << "(" << bbs.bounding_boxes[coreNumber].xmax << "," << bbs.bounding_boxes[coreNumber].ymax << ")" 
                                    << " : " << searchNumber << "(" << bbs.bounding_boxes[searchNumber].xmin << "," << bbs.bounding_boxes[searchNumber].ymin << ")-"
                                    << "(" << bbs.bounding_boxes[searchNumber].xmax << "," << bbs.bounding_boxes[searchNumber].ymax << ") in" 
                                    << groupNumber << " group";
                    break;
            }
            cv::putText(bridgeImage->image, ss.str(), cv::Point(20, 45 + text_row * 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, getColorFromColorMap(5), 2, CV_AA);
            ++text_row;
        }
    }
    ss.str("");
    ss.clear(std::stringstream::goodbit);
    cv::rectangle(bridgeImage->image, cv::Point(bbs.bounding_boxes[coreNumber].xmin, bbs.bounding_boxes[coreNumber].ymin), 
                  cv::Point(bbs.bounding_boxes[coreNumber].xmax, bbs.bounding_boxes[coreNumber].ymax), getColorFromColorMap(groupNumber-1), 5, 8);
    ss << "N" << coreNumber << " : G" << groupNumber;
    cv::putText(bridgeImage->image, ss.str(), cv::Point(bbs.bounding_boxes[coreNumber].xmin, bbs.bounding_boxes[coreNumber].ymin), cv::FONT_HERSHEY_SIMPLEX,
    0.7, getColorFromColorMap(groupNumber-1), 2, CV_AA);
    //drawMask(bbs, coreNumber, (char)groupNumber, mask);
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