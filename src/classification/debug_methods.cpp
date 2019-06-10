#include<obstacle_detection_2019/classification.h>


void classificationClass::showSearchWindows(){
    cv::Mat view= cv::Mat::zeros(smdCamera.widthInt.data, smdCamera.heightInt.data,CV_8UC3);
    //マップ対角線の半分の距離
    float halfLen = std::sqrt(smdCamera.widthInt.data*smdCamera.widthInt.data + smdCamera.heightInt.data*smdCamera.heightInt.data);
    cv::Point2i sp[9];
    //描画対象9つの座標を設定
    for(int k=0; k<9; k++){
        //y軸を0度
        int deg = minCamDeg + winDivDeg/2 + winDivDeg * k;
		float theta = (float)(deg)/180.0*M_PI;
        //中心からの相対座標->画像座標
        sp[k].x = (int)(halfLen * sin(theta)) + smdCamera.widthInt.data;
        sp[k].y = (int)(halfLen * cos(theta)) + smdCamera.heightInt.data/2;
    }
    //9つの座標に対する, 探索窓を描画
    for(int k=0; k<9; k++){
        //角度算出 と x軸=0度 -> y軸=0度に回転
        int angle = (int)( atan2(sp[k].y,sp[k].x)/M_PI *180) -90;
        //使用窓番号選択
        int num = selectWindow(angle);
        //窓内を探索
        for(int m=0; m < winIndex2[num].size(); m++){
            int w = sp[k].x + winIndex2[num][m] % smdCamera.widthInt.data;
            int h = sp[k].y + winIndex2[num][m] / smdCamera.widthInt.data;
            
            if(w < 0 || w >= smdCamera.widthInt.data
                || h < 0 || h >= smdCamera.heightInt.data ){
                    //マップ範囲外検索
                    continue;
            }
            //Matに描画
            uint8_t b = 255/9 * k;
            uint8_t g = 200;
            uint8_t r = 200;
            view.at<cv::Vec3b>(h, w)[0] = b;
            view.at<cv::Vec3b>(h, w)[1] = g;
            view.at<cv::Vec3b>(h, w)[2] = r;
        }
    }
    //表示データのPublish
    //cvBridgeデータ作成
	cv_bridge::CvImagePtr viewCvData(new cv_bridge::CvImage);
	viewCvData->encoding=sensor_msgs::image_encodings::BGR8;
    //データコピー
	viewCvData->image=view.clone();
    //Publish
	pubDeb.publish(viewCvData->toImageMsg());
}