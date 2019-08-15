#include<obstacle_detection_2019/convCamData.h>

//subscribe
bool convCamDataClass::subscribeSensorData(){//データ受信
    ros::CallbackQueue::CallOneResult res;
	res = queue.callOne(ros::WallDuration(1));
    ROS_INFO_STREAM("subscribe result : " << res);
    if(res == ros::CallbackQueue::CallOneResult::Called){return true;}
    return false;
}

void convCamDataClass::sensor_callback(const sensor_msgs::ImageConstPtr& msg)
{
    try{
        bridgeImage=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch(cv_bridge::Exception& e) {//エラー処理
        std::cout<<"depth_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.",
        msg->encoding.c_str());
        return ;
    }
    sensorData_subscribed_flag = true; //sta
    //追加
    manage();
}

void convCamDataClass::configCallback(obstacle_detection_2019::convCamDataConfig &config, uint32_t level) {
	ROS_INFO("Reconfigure Request: %d %f %f %f %f %f %f", 
		config.ransacNum, config.ransacDistanceThreshold,
		config.ransacEpsAngle, config.estimateCandidateY,
		config.estimateCameraHeight, config.estimateGroundThreshold,
		config.estimateHeightThreshold
		);
    //ransacパラメータ
    ransacNum = config.ransacNum;
    distanceThreshold = config.ransacDistanceThreshold;
    epsAngle = config.ransacEpsAngle;
    //ground パラメータ
    groundCandidateY = config.estimateCandidateY;
    camHeight = config.estimateCameraHeight;
    ground_th = config.estimateGroundThreshold;
    height_th = config.estimateHeightThreshold;
}
//manage method
void convCamDataClass::manage(){
    groundEstimationRANSAC();
    createPubDataRANSAC();
    publishConvCamData();
    publishMaskImage();
}

//床面推定
void convCamDataClass::groundEstimationRANSAC(){
    //groundCandidateY:床面候補点抽出用の閾値（高さ）
    //camHeight:カメラの高さ
    //a,b,c,d: ax+by+cz+d=d 床面の式
    //床面推定用のポイントクラウド変数
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZ>);
    //仮変数
    float x_temp;
    float y_temp;
    float z_temp;
    pcl::PointXYZ p_temp;
    int k=0;//床面候補点の数(ground_pointsのサイズ)
    int hMax = bridgeImage->image.rows;//行
    int wMax = bridgeImage->image.cols;//列
    ground_points->points.resize(hMax/2*wMax);//候補点の最大値でリサイズ
    //画素高速アクセス用パラメータ(画像チャンネル数)
    int ch = bridgeImage->image.channels();
    //床面候補点抽出処理
    for(int h=hMax/2+1;h<hMax;h++){//画像下半分を走査
        //画素高速アクセス用パラメータ(先頭アドレス)
        float *p = bridgeImage->image.ptr<float>(h);//ポインタ取得
        for(int w=0;w<wMax;w++){
            // z_temp=depth_image.at<float>(h,w);
            z_temp = p[w*ch];//上行と同じ内容
            if(z_temp>0.5&&!std::isinf(z_temp)){
                y_temp=((float)hMax/2-h)*z_temp/f;//高さ算出 ■この計算式どこから？
                if(std::abs(y_temp+camHeight)<groundCandidateY){//高さがgroundCandidateY未満の時
                    x_temp=-( ((float)w-(float)wMax/2)*z_temp/f-camHeight );
                    //座標系の変換
                    p_temp.x=z_temp;
                    p_temp.y=x_temp;
                    p_temp.z=y_temp;
                    // ground_points->points.push_back(p_temp);
                    ground_points->points[k++] = p_temp;
                }
            }
        }
    }
    //再度リサイズ
    ground_points->points.resize(k);//
    ground_points->width=ground_points->points.size();
    ground_points->height=1;
    ROS_INFO_STREAM("ground_points->points.size():"<<ground_points->points.size()<<"\n");
    //床面式の算出
    seg.setInputCloud (ground_points);
	seg.segment (*inliers, *coefficients);
    a=coefficients->values[0];
    b=coefficients->values[1];
    c=coefficients->values[2];
    d=coefficients->values[3];
    ROS_INFO_STREAM("Model coefficients: " << a << " "
                                        << b << " "
                                        << c << " "
                                        << d << "\n");
}
//
void convCamDataClass::createPubDataRANSAC(){
    //ステレオカメラパラメータ
    int hMax = bridgeImage->image.rows;//行
    int wMax = bridgeImage->image.cols;//列
    int ch = bridgeImage->image.channels();//チャンネル数
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
	std::cout<<a<<" x + "<<b<<" y + "<<c<<" z + "<<d<<" = 0\n";
    //仮変数
    float x_temp;
    float y_temp;
    float z_temp;
    //送信データ
    //マップデータ
    //
    smd.header = bridgeImage->header;
    smd.width.data = mapW;
    smd.height.data = mapH;
    smd.res.data = mapR;
    smd.widthInt.data = (int)(mapW/mapR);
    smd.heightInt.data = (int)(mapH/mapR);
    smd.cp.x=0;
    smd.cp.y=0;
    smd.cp.z=0;
    smd.index.resize(smd.heightInt.data*smd.widthInt.data);
    smd.size.resize(smd.heightInt.data*smd.widthInt.data);
    smd.pt.resize(smd.heightInt.data*smd.widthInt.data);
    // smd.index.resize(hMax*wMax);
    // smd.pt.resize(hMax*wMax);
    //マスクデータ
    mid.header = bridgeImage->header;
    //--画像データ
    mid.width.data = wMax;
    mid.height.data = hMax;
    //--マップデータ
    mid.mapWidth.data = mapW;
    mid.mapHeight.data = mapH;
    mid.mapRes.data = mapR;
    mid.mapWidthInt.data = (int)(mapW/mapR);
    mid.mapHeightInt.data = (int)(mapH/mapR);
    mid.pt.resize(wMax*hMax);
    //--インデックスデータ(maskImageData)
    // 画像ピクセル位置 -> マップデータへの対応付け
    // 画像ピクセル(h,w) -> マップデータ
    // index ==-1 のとき、対応するデータがないことを示す
    // 参照方法
    // pt[ index[width*h+w] ]
    //
    mid.index.resize(wMax*hMax);
    //--インデックスデータ(sensorMapData)
    //インデックス初期化
    for(int h=0;h<smd.heightInt.data;h++){
        for(int w=0;w<smd.widthInt.data;w++){
            smd.index[h*smd.widthInt.data+w].data=-1;
            smd.size[h*smd.widthInt.data+w].data=0;
            smd.pt[h*smd.widthInt.data+w].x = 0;
            smd.pt[h*smd.widthInt.data+w].y = 0;
            smd.pt[h*smd.widthInt.data+w].z = 0;
        }
    }
    //
    int countMid=0;
    int countSmd=0;
    //2次元ローカルマップの作成
    for(int h=0;h<hMax;h++){//画像すべてを走査
        //画素高速アクセス用パラメータ(先頭アドレス)
        float *p = bridgeImage->image.ptr<float>(h);//ポインタ取得
        for(int w=0;w<wMax;w++){
            z_temp = p[w*ch];//
            if(z_temp>0.5&&!std::isinf(z_temp)){
                y_temp=((float)hMax/2-h)*z_temp/f;//高さ算出
                x_temp=-( ((float)w-(float)wMax/2)*z_temp/f-camHeight );
                float y_ground=(-a*z_temp-b*x_temp-d)/c;//床面の高さを算出
                //高さが床面以上height_th以下の範囲を検出
                if(y_temp-y_ground<=0||y_temp-y_ground>height_th){
                    //マスクデータ格納
                    mid.index[h*wMax+w].data = -1;
                }
                else{
                    //データ格納
                    int xi,zi;//仮変数
                    if(convertToGrid(x_temp,z_temp,xi,zi)){//データ変換
                        //インデックスデータ
                        int index = smd.index[zi*smd.widthInt.data+xi].data;
                        //データがすでに格納されているか
                        if(index < 0){
                            //データがまだ無い場合
                            smd.index[zi*smd.widthInt.data+xi].data = countSmd;
                            index = countSmd++;//データ数をインクリメント, indexをsmd.index配列の末尾に設定 
                        }
                        //データサイズのインクリメント
                        smd.size[index].data+=1;
                        //マップデータ格納
                        smd.pt[index].x += x_temp;//横方向
                        smd.pt[index].y += z_temp;//奥行
                        smd.pt[index].z += y_temp;//高さ
                        //
                        //マスクデータ格納
                        mid.index[h*wMax+w].data = countMid;
                        mid.pt[countMid].x=x_temp;//横方向
                        mid.pt[countMid].y=z_temp;//奥行
                        mid.pt[countMid].z=y_temp;//高さ
                        //Midのデータ数をインクリメント
                        countMid++;
                    }
                    else{
                        //マスクデータ格納
                        mid.index[h*wMax+w].data = -1;
                    }
                }
            }
            else{
                //マスクデータ格納
                mid.index[h*wMax+w].data = -1;
            }
        }
    }
    //サイズ調整
    // smd.index.resize(k);
    smd.pt.resize(countSmd);
    smd.size.resize(countSmd);
    mid.pt.resize(countMid);
    //smdデータptの平均値を算出
    for(int k = 0; k < smd.size.size(); k++){
        if(smd.size[k].data > 0){
            //データサイズで割る
            smd.pt[k].x /= smd.size[k].data;//横方向
            smd.pt[k].y /= smd.size[k].data;//奥行
            smd.pt[k].z /= smd.size[k].data;//高さ
        }
    }
}
bool convCamDataClass::convertToGrid(const float& x,const float& y,int& xg,int& yg){
	//マップ上の中心座標(ふつうは　センサ位置＝マップ中心座標　のため　cx=cy=0)
	float cx=0;
	float cy=0;
    //マップ原点座標を画像原点座標(左上)に移動
	float map_x = mapW/2 + (x - cx);
	float map_y = mapH/2 + ( -(y - cy) );
    //マップ外のデータの際はreturn false
	if(map_x<0 || map_x>mapW)
		return false;
	if(map_y<0 || map_y>mapH)
		return false;
    //ピクセルデータに変換
	xg = (int)(map_x/mapR);
	yg = (int)(map_y/mapR);
    //変換成功
	return true;
}
void convCamDataClass::publishConvCamData(){//データ送信
    pubConv.publish(smd);
}
void convCamDataClass::publishMaskImage(){//データ送信
    pubMask.publish(mid);
}
void convCamDataClass::clearMessages(){
    smd.index.clear();
    smd.pt.clear();
    smd.size.clear();
    mid.index.clear();
    mid.pt.clear();
}

bool convCamDataClass::is_SensorData_subscribed(){
    return sensorData_subscribed_flag;
}