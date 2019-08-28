#include<obstacle_detection_2019/imageMatching.h>

//デバッグ用メソッド
void imageMatchingClass::debug(){
    switch(debugType){
        case 1: showMatchingMap();break;
        case 2: showMatchingImage();break;
        default: ;
    }
}
//オプティカルフロー結果(マッチング結果)を２次元マップ上に示す
void imageMatchingClass::showMatchingMap(){
    // matchDataを使用(ImageMatchingData.msg)
    int width = matchData.widthInt.data;//画像横幅
    int height = matchData.heightInt.data;//画像縦幅    
    //
    //出力表示用Matデータ、0で初期化、RGB　8bit
    cv::Mat viewData = cv::Mat::zeros(cv::Size(width,height),CV_8UC3);

    //matchDataの走査と矢印描画
    // w = k % width (k -> w)
    // h = k / width (k -> h)
    // k = h * width + w (w,h -> k)
    for(int k = 0; k < width*height; k++){        
        //マッチデータがマップ(h,w)に存在しない時スキップ
        if(matchData.index[k].data < 0){
            continue;
        }
        //マッチングデータがある場合は矢印を描画
        //k -> h,w 
        int h = k / width;
        int w = k % width;
        //参照先データ番号
        int dataNum = matchData.index[k].data;
        //マッチング先までの偏差
        int matchX = matchData.data[dataNum].x.data;
        int matchY = matchData.data[dataNum].y.data;
        // ROS_INFO("h,w,mx,my:%d,%d,%d,%d",h,w,matchX,matchY);
        //始点終点
        cv::Point2i p1 = cv::Point2i(w,h);//始点
        cv::Point2i p2 = cv::Point2i(w + matchX, h + matchY);//終点
        cvArrow(&viewData, p1, p2);
    }
    //表示データのPublish
    //cvBridgeデータ作成
	cv_bridge::CvImagePtr viewCvData(new cv_bridge::CvImage);
	viewCvData->encoding=sensor_msgs::image_encodings::BGR8;
    //データコピー
	viewCvData->image=viewData.clone();
    //Publish
	pubDeb.publish(viewCvData->toImageMsg());
}
void imageMatchingClass::showMatchingImage(){
    // matchDataを使用(ImageMatchingData.msg)
    int width = matchData.widthInt.data;//画像横幅
    int height = matchData.heightInt.data;//画像縦幅    
    //
    //出力表示用Matデータ、0で初期化、RGB　8bit
    cv::Mat viewData = bridgeImageCur->image.clone();

    //matchDataの走査と矢印描画
    // w = k % width (k -> w)
    // h = k / width (k -> h)
    // k = h * width + w (w,h -> k)
    for(int k = 0; k < featurePointsCur.size(); k++){        
        //始点終点
        int cw = (int)featurePointsCur[k].x;
        int ch = (int)featurePointsCur[k].y;
        int pw = (int)featurePointsPre[k].x;
        int ph = (int)featurePointsPre[k].y;
        cv::Point2i p1 = cv::Point2i(pw,ph);//始点
        cv::Point2i p2 = cv::Point2i(cw, ch);//終点
        cvArrow(&viewData, p1, p2);
    }
    //表示データのPublish
    //cvBridgeデータ作成
	cv_bridge::CvImagePtr viewCvData(new cv_bridge::CvImage);
	viewCvData->encoding=sensor_msgs::image_encodings::BGR8;
    //データコピー
	viewCvData->image=viewData.clone();
    //Publish
	pubDeb.publish(viewCvData->toImageMsg());
}
//cvArrow説明
// cv::Mat* img : 描画対象Mat
// cv::Point2i pt1 : 矢印の始点
// cv::Point2i pt2 : 矢印の終点
// cv::Scalar color : カラー(BGR)
// int thickness : 太さ
// int lineType : 線の種類
// int shift : 忘れた
//
void imageMatchingClass::cvArrow(cv::Mat* img, cv::Point2i pt1, cv::Point2i pt2, cv::Scalar color, int thickness, int lineType, int shift){

  cv::line(*img,pt1,pt2,color,thickness,lineType,shift);
  float vx = (float)(pt2.x - pt1.x);
  float vy = (float)(pt2.y - pt1.y);
  float v = sqrt( vx*vx + vy*vy );
  float ux = vx / v;
  float uy = vy / v;
  //矢印の幅の部分
  float w=5,h=10;
  cv::Point2i ptl,ptr;
  ptl.x = (int)((float)pt2.x - uy*w - ux*h);
  ptl.y = (int)((float)pt2.y + ux*w - uy*h);
  ptr.x = (int)((float)pt2.x + uy*w - ux*h);
  ptr.y = (int)((float)pt2.y - ux*w - uy*h);
  //矢印の先端を描画する
  //--例外処理(!(v==0)) : ベクトルの大きさがゼロの時->点のみを表示
  if(!(v==0)){
      cv::line(*img,pt2,ptl,color,thickness,lineType,shift);
      cv::line(*img,pt2,ptr,color,thickness,lineType,shift);
  }
}