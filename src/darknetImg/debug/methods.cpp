#include <obstacle_detection_2019/darknetImgDebug.h>

void darknetImgDebug::debug_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bb,const sensor_msgs::Image::ConstPtr& image){
    ROS_INFO_STREAM("debug callback : " << "[bb : " << bb->header.stamp.toSec() << " ]," 
                                  << "[img : " << image->header.stamp.toSec() << " ],"
                                  << "[diff : " << (bb->header.stamp - image->header.stamp).toSec() << "]");
    // image から cvbridgeImage に変換する
    try{
        // 枠線描画テスト・BoundingBoxesのテスト のときはこちら
        // bridgeImage=cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);
        // 実際の処理
        bridgeImage=cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch(cv_bridge::Exception& e) {//エラー処理
        ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.",image->encoding.c_str());
        return;
    }
    boundingBoxesMsg = *bb;

    // 枠線描画テスト
    // bounding_boxedに書かれた枠を描画する
    // auto iter = bb->bounding_boxes.begin();
    // for (; iter != bb->bounding_boxes.end(); ++iter){
    //     cv::rectangle(bridgeImage->image, cv::Point(iter->xmin, iter->ymin), cv::Point(iter->xmax, iter->ymax), cv::Scalar(0, 0, 200), 5, 8);
    // }
    // pub.publish(bridgeImage->toImageMsg());

    //boundingBoxesのテスト
    // trimPoints(boundingBoxesMsg);
    // cv_bridge::CvImage cvMask;
    // cvMask.header = bridgeImage->header;
    // cvMask.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    // cvMask.image = mask.clone();
    // bbMaskImage_pub.publish(cvMask.toImageMsg());
    // bbImage_pub.publish(bridgeImage->toImageMsg());
    
    // 実際の処理
    if (bb->bounding_boxes.size() > 0){
        depth2points();
        pickUpGroundPointCandidates();
        estimateGroundCoefficients();
        removeGroundPoints();
        trimPoints(boundingBoxesMsg);
        publishTrimMask();
        for(int row=0; row < bridgeImage->image.rows; row++){
            auto mi = mask.ptr<char>(row);
            for(int col=0; col < bridgeImage->image.cols; col++){
                if((int)(mi[col]) > detection_total){
                    ROS_WARN_STREAM("over detection total ... " << row << " " << col << " " << mi[col] << " " << (int)mi[col] << " " << detection_total);
                }
            }
        }
        generateGridMap();
        publishGridMap();
        // generateGridmap();
        // publishGridMap();
        // ROS_INFO_STREAM("pickUpGroundPointCandidates");
        // pickUpGroundPointCandidates();
        // ROS_INFO_STREAM("estimateGroundCoefficients");
        // estimateGroundCoefficients();
        // ROS_INFO_STREAM("removeGroundPoints");
        // removeGroundPoints();
        // ROS_INFO_STREAM("trimPoints");
        // trimPoints(boundingBoxesMsg);
        // ROS_INFO_STREAM("generateGridmap");
        // generateGridmap();
        // ROS_INFO_STREAM("dimensionalityReductionGridmap");
        // dimensionalityReductionGridmap();
        // ROS_INFO_STREAM("classifyPoints");
        // classifyPoints();
        // ROS_INFO_STREAM("estimatePersonPosition");
        // estimatePersonPosition();
        // ROS_INFO_STREAM("predictPersonPosition");
        // predictPersonPosition();
        // ROS_INFO_STREAM("iteration finished");
        // cluster2PointCloud(cd, pcl_msg);
        // pcl_pub.publish(pcl_msg);
    }else{
        ROS_INFO_STREAM("not person detection");
    }

    // clearMsg(smdml);
    //ROS_INFO_STREAM("----------------------------------------");
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