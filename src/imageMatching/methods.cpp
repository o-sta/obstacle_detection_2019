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
void imageMatchingClass::maskImage_callback(const obstacle_detection_2019::MaskImageData::ConstPtr& msg)
{
    midCur.header = msg->header;
    //--画像データ
    midCur.width.data = msg->width.data;
    midCur.height.data = msg->height.data;
    //--マップデータ
    midCur.mapWidth.data = msg->mapWidth.data;
    midCur.mapHeight.data = msg->mapHeight.data;
    midCur.mapRes.data = msg->mapRes.data;
    midCur.mapWidthInt.data = msg->mapWidthInt.data;
    midCur.mapHeightInt.data = msg->mapHeightInt.data;
    midCur.index = msg->index;
    midCur.pt = msg->pt;
    
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
bool imageMatchingClass::isMaskImagePre(){
    if(midPre.pt.empty()){//データの有無を確認
        return false;
    }
    else{
        return true;
    }
}
void imageMatchingClass::resetData(){
    // *bridgeImagePre = *bridgeImageCur;
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
// ■初期化
// void imageMatchingClass::getFeaturePointNum(){

// }
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
                    if(midCur.index[pt.y*midCur.width.data + pt.x].data >= 0){
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
        if(midCur.index[(int)featurePointsCur[k].y*midCur.width.data + (int)featurePointsCur[k].x].data>=0 && sts[k]==1){
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
    matchData.index.resize(matchData.widthInt.data*matchData.heightInt.data);
    matchData.data.resize((int)featurePointsCur.size());
    int dataSize=0;//データサイズカウント

    //インデックスの中身を初期化
    for(int k=0;k<matchData.index.size();k++){
        matchData.index[k].data = -1;
    }
    //
    for(int k=0;k<featurePointsCur.size();k++){
        //時刻tのデータ
        int cW = (int)featurePointsCur[k].x;
        int cH = (int)featurePointsCur[k].y;
        int cIndex = midCur.index[cH*midCur.width.data + cW].data;
        geometry_msgs::Point cPt =  midCur.pt[cIndex];
        //時刻t-Delta_tのデータ
        int pW = (int)featurePointsPre[k].x;
        int pH = (int)featurePointsPre[k].y;
        int pIndex = midPre.index[pH*midPre.width.data + pW].data;
        geometry_msgs::Point pPt =  midPre.pt[pIndex];
        //マップ位置
        int xi,yi,zi;//■ ziを追加した
        if(convertToGrid(cPt.x,cPt.y,xi,zi)){//データ変換　■ここに問題が？
            //移動量: cur - pre
            float difX=cPt.x - pPt.x;
            float difY=cPt.y - pPt.y;
            //インデックスデータ
            matchData.index[zi*matchData.widthInt.data+xi].data = dataSize; //■わからないので、smdからmatchDataに変更した
            //マップ上の移動量
            matchData.data[dataSize].x.data = (int)(difX/midCur.mapRes.data);
            matchData.data[dataSize].y.data = -(int)(difY/midCur.mapRes.data);
            //実測の移動角度
            matchData.data[dataSize].theta.data = std::atan2(difY,difX);//-PIから+PI ■引数２つ必要　勝手に変更した difY/difX から difY,difX
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