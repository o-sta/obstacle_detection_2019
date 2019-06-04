#include<obstacle_detection_2019/convPCLData.h>


//subscribe
void convPCLDataClass::subscribeSensorData(){//データ受信
	queue.callOne(ros::WallDuration(1));
}
void convPCLDataClass::sensor_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    cloud2Data = *msg;//要確認
    sensorData_subscribed_flag = true; //sta
}
void convPCLDataClass::convCloud2toPCL(){
    pcl::fromROSMsg(cloud2Data,pclData);
}
void convPCLDataClass::convPCLtoImages(){
    //画像Mat
    cv::Mat rgbImage,depthImage;
    //サイズ取得
    int width = pclData.width;
    int height = pclData.height;
    //画像初期化
    rgbImage = cv::Mat::zeros(cv::Size(width,height),CV_8UC3);
    depthImage = cv::Mat::zeros(cv::Size(width,height),CV_32FC1);//depthのみ
    int chRgb=rgbImage.channels();//チャンネル数(今のところ未使用)
    int chDepth=depthImage.channels();//チャンネル数(今のところ未使用)
    //画像データ作成
    for(int h = 0; h < height; h++){
        cv::Vec3b *pRgb = rgbImage.ptr<cv::Vec3b>(h);
        cv::Vec3f *pDepth = depthImage.ptr<cv::Vec3f>(h);
        for(int w = 0; w < width; w++){
            int k = h * width + w;
            pRgb[w][0] = pclData.points[k].b;//たぶんBlue (BGR)
            pRgb[w][1] = pclData.points[k].g;//たぶんGleen (BGR)
            pRgb[w][2] = pclData.points[k].r;//たぶんRed (BGR)
            pDepth[w] = pclData.points[k].x;//X depth 今回は奥行きのみを使用
            // pDepth[w][0] = pclData.points[k].x;//X y
            // pDepth[w][1] = -pclData.points[k].Y;//Y -x
            // pDepth[w][2] = pclData.points[k].Z;//Z z
        }
    }
    //ROSヘッダ
    bridgeRgb->header = cloud2Data.header;
    bridgeDepth->header = cloud2Data.header;
    //エンコーダ
    bridgeRgb->encoding=sensor_msgs::image_encodings::BGR8;
    bridgeDepth->encoding=sensor_msgs::image_encodings::TYPE_32FC1;
    //画像コピー
    bridgeRgb->image = rgbImage.clone();
    bridgeDepth->image = depthImage.clone();
}
//publish
void convPCLDataClass::publishRgbCamData(){//データ送信
    pubRgb.publish(bridgeRgb->toImageMsg());
}
void convPCLDataClass::publishDepthCamData(){//データ送信
    pubDepth.publish(bridgeDepth->toImageMsg());
}
