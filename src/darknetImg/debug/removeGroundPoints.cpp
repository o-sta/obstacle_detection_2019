#include <obstacle_detection_2019/darknetImgDebug.h>

void darknetImgDebug::depth2points(){
    pcl::PointCloud<pcl::PointXYZRGB> ground_points_ex;
    float xt, yt, zt; //点の座標（テンプレート）
    pcl::PointXYZRGB pt; //点の座標（ポイントクラウド型テンプレート）
    int rows = bridgeImage->image.rows; //深度画像の行
    int cols = bridgeImage->image.cols; //深度画像の列
    int candidateNum = 0;//床面候補点の数(ground_pointsのサイズ)
    ground_points_ex.points.resize(rows*cols);//候補点の最大値でリサイズ
    int ch = bridgeImage->image.channels(); //チャンネル数
    //床面候補点抽出処理
    for(int i = 0; i<rows; i++){//画像下半分を走査
        float *p = bridgeImage->image.ptr<float>(i); //i行1列目のアドレス
        for(int j = 0; j < cols; j++){//走査}
            zt = p[j*ch];
            if(zt > 0.5 && !std::isinf(zt)){
                yt = ((float)rows/2-i)*zt/f; //高さ
                xt = -( ((float)i-(float)cols/2)*zt/f-camHeight );
                pt.x=zt;
                pt.y=xt;
                pt.z=yt;
                pt.r=colorMap[1];
                pt.g=colorMap[2];
                pt.b=colorMap[3];
                ground_points_ex.points[candidateNum++] = pt;
            }
        }
    }
    ground_points_ex.points.resize(candidateNum);
    ground_points_ex.width=ground_points_ex.points.size();
    ground_points_ex.height=1;
    pcl::toROSMsg (ground_points_ex, depthPCL_msg);
    depthPCL_msg.header.frame_id="/zed_camera_center";
    depth2points_pub.publish(depthPCL_msg);
    ROS_INFO_STREAM("ground_points->points.size():"<<ground_points_ex.points.size()<<"\n");
    
}

void darknetImgDebug::pickUpGroundPointCandidates(){
    float xt, yt, zt; //点の座標（テンプレート）
    pcl::PointXYZ pt; //点の座標（ポイントクラウド型テンプレート）
    int rows = bridgeImage->image.rows; //深度画像の行
    int cols = bridgeImage->image.cols; //深度画像の列
    int candidateNum = 0;//床面候補点の数(ground_pointsのサイズ)
    ground_points->points.resize(rows/2*cols);//候補点の最大値でリサイズ
    int ch = bridgeImage->image.channels(); //チャンネル数
    //床面候補点抽出処理
    for(int i = rows/2+1; i<rows; i++){//画像下半分を走査
        float *p = bridgeImage->image.ptr<float>(i); //i行1列目のアドレス
        for(int j = 0; j < cols; j++){//走査}
            zt = p[j*ch];
            if(zt > 0.5 && !std::isinf(zt)){
                yt = ((float)rows/2-i)*zt/f; //高さ
                if(std::abs(yt+camHeight) < groundCandidateY){ //高さがgroundCandidateY未満の時
                    xt = -( ((float)i-(float)cols/2)*zt/f-camHeight );
                    pt.x=zt;
                    pt.y=xt;
                    pt.z=yt;
                    ground_points->points[candidateNum++] = pt;
                }
            }
        }
    }
    ground_points->points.resize(candidateNum);
    ground_points->width=ground_points->points.size();
    ground_points->height=1;
    pcl::toROSMsg(*ground_points, groundCanPCL_msg);
    groundCanPCL_msg.header.frame_id="/zed_camera_center";
    pickUpGroundPointCandidates_pub.publish(groundCanPCL_msg);
    ROS_INFO_STREAM("ground_points->points.size():"<<ground_points->points.size()<<"\n");
}

void darknetImgDebug::estimateGroundCoefficients(){
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
}


void darknetImgDebug::removeGroundPoints(){
    int row = 0, col = 0;
    float xt, yt, zt;
    int ch = bridgeImage->image.channels();
    int rows = bridgeImage->image.rows; //深度画像の行
    int cols = bridgeImage->image.cols; //深度画像の列
    //マスクのりサイズ
    if(is_size_initialized == false){
        cv::resize(mask, mask, cv::Size(rows,cols));
    }
    for(row = 0; row < rows; row++){
        float *bi = bridgeImage->image.ptr<float>(row);
        char *mi = mask.ptr<char>(row);
        for(col = 0; col < cols; col++){
            zt = bi[col*ch];
            if(zt>0.5&&!std::isinf(zt)){
                yt=((float)cols/2-row)*zt/f;//高さ算出
                //xt=-( ((float)row-(float)rows/2)*zt/f-camHeight );
                float y_ground=(-a*zt-b*xt-d)/c;//床面の高さを算出
                //高さが床面以上であればmask値を1に
                yt > y_ground ? mi[col] = 1 : mi[col] = 0;
            } else{
                mi[col] = 0;
            }
        }
    }
}
