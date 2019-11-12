#include <obstacle_detection_2019/darknetImgDebug.h>

void darknetImgDebug::depth2points(){
    // pcl::PointCloud<pcl::PointXYZRGB> ground_points_ex;
    float xt, yt, zt; //点の座標（テンプレート）
    pcl::PointXYZRGB pt; //点の座標（ポイントクラウド型テンプレート）
    int rows = bridgeImage->image.rows; //深度画像の行
    int cols = bridgeImage->image.cols; //深度画像の列
    int candidateNum = 0;//床面候補点の数(ground_pointsのサイズ)
    depth_points->points.resize(rows*cols);//候補点の最大値でリサイズ
    int ch = bridgeImage->image.channels(); //チャンネル数
    float temp_rand;
    //床面候補点抽出処理
    for(int i = 0; i<rows; i++){//画像下半分を走査
        float *p = bridgeImage->image.ptr<float>(i); //i行1列目のアドレス
        for(int j = 0; j < cols; j++){//走査}
            zt = p[j*ch];
            if(zt > 0.5 && !std::isinf(zt)){
                yt = ((float)rows/2-(float)i)*zt/f+camHeight; //高さ
                xt = -( ((float)j-(float)cols/2)*zt/f );
                // temp_rand = 0.3*((float)std::rand()/(float)RAND_MAX);
                pt.x=zt;
                pt.y=xt;
                pt.z=yt;
                pt.r=colorMap[0];
                pt.g=colorMap[1];
                pt.b=colorMap[2];
                depth_points->points[candidateNum++] = pt;
            }
        }
    }
    depth_points->points.resize(candidateNum);
    depth_points->width=depth_points->points.size();
    depth_points->height=1;
    pcl::toROSMsg (*depth_points, depthPCL_msg);
    // depthPCL_msg.header.frame_id="/zed_camera_center";
    depthPCL_msg.header.frame_id="/zed_camera_center";
    depth2points_pub.publish(depthPCL_msg);
    ROS_INFO_STREAM("ground_points->points.size():"<<depth_points->points.size()<<"\n");
}

void darknetImgDebug::pickUpGroundPointCandidates(){
    pcl::PointCloud<pcl::PointXYZRGB> ground_points_temp;
    pcl::PointXYZRGB pt_ex; //点の座標（ポイントクラウド型テンプレート）
    float xt, yt, zt; //点の座標（テンプレート）
    pcl::PointXYZ pt; //点の座標（ポイントクラウド型テンプレート）
    int rows = bridgeImage->image.rows; //深度画像の行
    int cols = bridgeImage->image.cols; //深度画像の列
    int candidateNum = 0;//床面候補点の数(ground_pointsのサイズ)
    ground_points->points.resize(rows/2*cols);//候補点の最大値でリサイズ
    ground_points_temp.points.resize(rows/2*cols);
    int ch = bridgeImage->image.channels(); //チャンネル数
    //床面候補点抽出処理
    for(int i = rows/2+1; i<rows; i++){//画像下半分を走査
        float *p = bridgeImage->image.ptr<float>(i); //i行1列目のアドレス
        for(int j = 0; j < cols; j++){//走査}
            zt = p[j*ch];
            if(zt > 0.5 && !std::isinf(zt)){
                yt = ((float)rows/2-i)*zt/f+camHeight; //高さ
                if(std::abs(yt+camHeight) < groundCandidateY){ //高さがgroundCandidateY未満の時
                    xt = -( ((float)j-(float)cols/2)*zt/f);
                    pt.x=zt;
                    pt.y=xt;
                    pt.z=yt;
                    ground_points->points[candidateNum++] = pt;
                    pt_ex.x=zt;
                    pt_ex.y=xt;
                    pt_ex.z=yt;
                    pt_ex.r=colorMap[3];
                    pt_ex.g=colorMap[4];
                    pt_ex.b=colorMap[5];
                    ground_points_temp.points[candidateNum] = pt_ex;
                }
            }
        }
    }
    ground_points->points.resize(candidateNum);
    ground_points->width=ground_points->points.size();
    ground_points->height=1;
    ground_points_temp.points.resize(candidateNum);
    ground_points_temp.width=ground_points_temp.points.size();
    ground_points_temp.height=1;
    pcl::toROSMsg(ground_points_temp, groundCanPCL_msg);
    groundCanPCL_msg.header.frame_id="/zed_camera_center";
    pickUpGroundPointCandidates_pub.publish(groundCanPCL_msg);
    ROS_INFO_STREAM("ground_points->points.size():"<<ground_points->points.size()<<"\n");
}

void darknetImgDebug::estimateGroundCoefficients(){
    pcl::PointCloud<pcl::PointXYZRGB> ground_points_temp;
    int gp_size;
    seg.setInputCloud(ground_points);
	seg.segment(*inliers, *coefficients);
    a=coefficients->values[0];
    b=coefficients->values[1];
    c=coefficients->values[2];
    d=coefficients->values[3];
    //床面式の確認と再設定
	//const float ground_th=0.20;//床面式から高さground_thまでのデータを床面とする
    if(std::abs(d-camHeight>=0.15)){//推定値があまりにも変な場合
        //事前に決めた値を使用
		a=-0.08;
		b=0;
		c=1;
		d=camHeight-ground_th;
	}
	else{
		d-=ground_th;
	}
    ROS_INFO_STREAM("Model coefficients: " << a << " "
                                    << b << " "
                                    << c << " "
                                    << d << "\n");

    gp_size = 0;
    ground_points_temp.points.resize(depth_points->size());
    for(auto index : inliers->indices){
        ground_points_temp.points[gp_size].x = ground_points->points[index].x;
        ground_points_temp.points[gp_size].y = ground_points->points[index].y;
        ground_points_temp.points[gp_size].z = ground_points->points[index].z;
        ground_points_temp.points[gp_size].r = colorMap[6];
        ground_points_temp.points[gp_size].g = colorMap[7];
        ground_points_temp.points[gp_size].b = colorMap[8];
        gp_size++;
    }
    ground_points_temp.points.resize(gp_size);
    ground_points_temp.width=ground_points_temp.points.size();
    ground_points_temp.height=1;
    pcl::toROSMsg(ground_points_temp, groundPCL_msg);
    groundPCL_msg.header.frame_id="/zed_camera_center";
    estimateGroundCoefficients_pub.publish(groundPCL_msg);
}

void darknetImgDebug::removeGroundPoints(){
    pcl::PointCloud<pcl::PointXYZRGB> ground_points_temp;
    int gp_size;
    int row = 0, col = 0;
    float xt, yt, zt;
    int ch = bridgeImage->image.channels();
    int rows = bridgeImage->image.rows; //深度画像の行
    int cols = bridgeImage->image.cols; //深度画像の列
    //マスクのりサイズ
    if(is_size_initialized == false){
        cv::resize(obstacle_mask, obstacle_mask, cv::Size(cols,rows));
    }
    ground_points_temp.points.resize(rows*cols);
    gp_size = 0;
    for(row = 0; row < rows; row++){
        float *bi = bridgeImage->image.ptr<float>(row);
        char *mi = obstacle_mask.ptr<char>(row);
        for(col = 0; col < cols; col++){
            zt = bi[col*ch];
            if(zt>0.5&&!std::isinf(zt)){
                yt=((float)rows/2-row)*zt/f+camHeight;//高さ算出
                xt = -( ((float)col-(float)cols/2)*zt/f);
                float y_ground=(-a*zt-b*xt-d)/c;//床面の高さを算出
                //高さが床面以上であればobstacle_mask値を1に
                yt > y_ground ? mi[col] = 1 : mi[col] = 0;
                if(yt > y_ground){
                    ground_points_temp.points[gp_size].x = zt;
                    ground_points_temp.points[gp_size].y = xt;
                    ground_points_temp.points[gp_size].z = yt;
                    ground_points_temp.points[gp_size].r = colorMap[9];
                    ground_points_temp.points[gp_size].g = colorMap[10];
                    ground_points_temp.points[gp_size].b = colorMap[11];
                    gp_size++;
                }
            } else{
                mi[col] = 0;
            }
        }
    }
    ground_points_temp.points.resize(gp_size);
    ground_points_temp.width=ground_points_temp.points.size();
    ground_points_temp.height=1;
    pcl::toROSMsg(ground_points_temp, obstaclePCL_msg);
    obstaclePCL_msg.header.frame_id="/zed_camera_center";
    removeGroundPoints_pub.publish(obstaclePCL_msg);
}
