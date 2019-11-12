#include <obstacle_detection_2019/darknetImg.h>


void darknetImg::pickUpGroundPointCandidates(){
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
                yt = ((float)rows/2-i)*zt/f + camHeight; //高さ
                if(std::abs(yt+camHeight) < groundCandidateY){ //高さがgroundCandidateY未満の時
                    xt = -( ((float)j-(float)cols/2)*zt/f );
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
    ROS_INFO_STREAM("ground_points->points.size():"<<ground_points->points.size()<<"\n");
}

void darknetImg::estimateGroundCoefficients(){
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


void darknetImg::removeGroundPoints(){
    int row = 0, col = 0;
    float xt, yt, zt;
    int ch = bridgeImage->image.channels();
    int rows = bridgeImage->image.rows; //深度画像の行
    int cols = bridgeImage->image.cols; //深度画像の列
    //マスクのりサイズ
    if(is_size_initialized == false){
        cv::resize(obstacle_mask, obstacle_mask, cv::Size(cols,rows));
    }
    for(row = 0; row < rows; row++){
        float *bi = bridgeImage->image.ptr<float>(row);
        char *mi = obstacle_mask.ptr<char>(row);
        for(col = 0; col < cols; col++){
            zt = bi[col*ch];
            if(zt>0.5&&!std::isinf(zt)){
                yt=((float)rows/2-row)*zt/f + camHeight;//高さ算出
                xt = -(((float)col-(float)cols/2)*zt/f);
                float y_ground=(-a*zt-b*xt-d)/c;//床面の高さを算出
                //高さが床面以上であればobstacle_mask値を1に
                yt > y_ground ? mi[col] = 1 : mi[col] = 0;
            } else{
                mi[col] = 0;
            }
        }
    }
}
