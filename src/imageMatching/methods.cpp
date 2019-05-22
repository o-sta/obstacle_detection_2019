#include<obstacle_detection_2019/imageMatching.h>

//subscribe
void imageMatchingClass::subscribeImageData(){//データ受信
	queue1.callOne(ros::WallDuration(1));
}
void imageMatchingClass::subscribeMaskImageData(){//データ受信
	queue2.callOne(ros::WallDuration(1));
}
void imageMatchingClass::image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    try{
        bridgeImageCur=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e) {//エラー処理
        std::cout<<"depth_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to 'BGR8'.",
        msg->encoding.c_str());
        return ;
    } 
}
void imageMatchingClass::cvtGray(){
    cv::cvtColor(bridgeImageCur->image,grayImgCur,CV_BGR2GRAY);
}
bool imageMatchingClass::isBridgeImagePre(){
    if(bridgeImagePre->image.empty()){//要注意
        return false;
    }
    else{
        return true;
    }
}
void imageMatchingClass::resetData(){
    // bridgeImagePre = bridgeImageCur;//これをしていいのか不明
    //念のためしっかりコピー
    bridgeImagePre->header = bridgeImageCur->header;
    bridgeImagePre->encoding = bridgeImageCur->encoding;
    bridgeImagePre->image = bridgeImageCur->image.clone();
    // maskImageData
    midPre = midCur;
    //グレースケール画像
    grayImgPre = grayImgCur.clone();
    //特徴点
    if((int)featurePointsCur.size()>0){
        featurePointsPre = featurePointsCur;
    }
    else{
        featurePointsPre = featurePointsCur;
    }
    featurePointsTemp.clear();
    featurePointsCur.clear();
}
void imageMatchingClass::getFeaturePointNum(){

}
void imageMatchingClass::getFeaturePoints(){
    //キーポイント
    std::vector<cv::KeyPoint> keypoints;
    //特徴点抽出器
    auto detector = cv::FastFeatureDetector::create(maxDetectPoint,false);
    //画像を分割して均等に特徴点を抽出
    int clipPixH=(int)grayImgCur.rows / nh;
    int clipPixW=(int)grayImgCur.cols / nw;
    //特徴点リサイズ
    featurePointsTemp.resize(maxPoint*nh*nw);
    int fpSize=0;//特徴点サイズカウンタ
    for(int h=0;h<nh;h++){
        for(int w=0;w<nw;w++){
            //画像分割
            clipImg=grayImgCur(cv::Rect(w*clipPixW,h*clipPixH,clipPixW,clipPixH));
            //キーポイントを抽出
            detector->detect(clipImg, keypoints);
            //レスポンスが強い順（降順）にソート
            sort(keypoints.begin(),keypoints.end(),
                [](const cv::KeyPoint& x, const cv::KeyPoint& y) {return  x.response > y.response;});
            //keypointの中から特徴点を抽出
            //各エリアごとの特徴点取得数をカウント
            int count=0;
            for(std::vector<cv::KeyPoint>::iterator itk = keypoints.begin();
                itk != keypoints.end() && count <= maxPoint; ++itk){
                    cv::Point2i pt;
                    pt.x=w*clipPixW+(int)itk->pt.x;
                    pt.y=h*clipPixH+(int)itk->pt.y;
                    if(midCur.index[pt.y*midCur.width + pt.x] >= 0){
                        featurePointsTemp[fpSize++] = pt;
                    }
            }
        }
    }
    //特徴点リサイズ
    featurePointsTemp.resize(fpSize);
}
void imageMatchingClass::featureMatching(){
    cv::calcOpticalFlowPyrLK(grayImgPre,grayImgCur, featurePointsPre, featurePointsCur, sts, ers,
        cv::Size(ws,ws),3,
        cvTermCriteria (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 30, 0.05), 1);
}
void imageMatchingClass::checkMatchingError(){
    //エラーチェックして再格納
    int count=0;//格納数カウント
    for(int k=0;k<featurePointsCur.size();k++){
        if(midCur.index[(int)featurePointsCur[k].y*midCur.width + (int)featurePointsCur[k].x]>=0 && sts[k]==1){
            featurePointsPre[count] = featurePointsPre[k];
            featurePointsCur[count] = featurePointsCur[k];
            count++;
        }
    }
    //リサイズ
    featurePointsPre.resize(count);
    featurePointsCur.resize(count);
}
void imageMatchingClass::creatMatchingData(){
    //送信データの設定
    matchData.header = midCur.header;
    matchData.width = midCur.mapWidth;
    matchData.height = midCur.mapHeight;
    matchData.res = midCur.mapRes;
    matchData.widthInt = midCur.mapWidthInt;
    matchData.heightInt = midCur.mapHeightInt;
    //--cpはlaunchファイルで設定すべき
    matchData.cp.x=0;
    matchData.cp.y=0;
    matchData.cp.z=0;
    //リサイズ
    matchData.index.resize(matchData.widthInt*matchData.heightInt);
    matchData.data.resize((int)featurePointsCur.size());
    int dataSize=0;//データサイズカウント

    //インデックスの中身を初期化
    for(int k=0;k<matchData.index.size();k++){
        matchData.index[k] = -1;
    }
    //
    for(int k=0;k<featurePointsCur.size();k++){
        //時刻tのデータ
        int cW = (int)featurePointsCur[k].x;
        int cH = (int)featurePointsCur[k].y;
        int cIndex = midCur.index[cH*midCur.width + cW];
        geometry_msgs::Point cPt =  midCur.pt[cIndex];
        //時刻t-Delta_tのデータ
        int pW = (int)featurePointsPre[k].x;
        int pH = (int)featurePointsPre[k].y;
        int pIndex = midPre.index[pH*midPre.width + pW];
        geometry_msgs::Point pPt =  midPre.pt[pIndex];
        //マップ位置
        int xi,yi;
        if(convertToGrid(cPt.x,cPt.y,xi,zi)){//データ変換
            //移動量: cur - pre
            float difX=cPt.x - pPt.x;
            float difY=cPt.y - pPt.y;
            //インデックスデータ
            matchData.index[zi*smd.widthInt+xi] = dataSize;
            //マップ上の移動量
            matchData.data[dataSize].x = (int)(difX/midCur.mapRes);
            matchData.data[dataSize].y = -(int)(difY/midCur.mapRes);
            //実測の移動角度
            matchData.data[dataSize].theta = std::atan2(difY/difX);//-PIから+PI
            //インクリメント
            dataSize++;
        }
    }
    //リサイズ
    matchData.data.resize(dataSize);
}

bool imageMatchingClass::convertToGrid(const float& x,const float& y,int& xg,int& yg){
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
void imageMatchingClass::publishMatchingData(){//データ送信
    pubMatch.publish(matchData);
}