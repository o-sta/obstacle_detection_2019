//多重インクルード防止
#ifndef INCLUDE_MANAGEMENT_CLASS
#define INCLUDE_MANAGEMENT_CLASS
//include haeders
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include<obstacle_detection_2019/synchronizedImage.h>

//クラスの定義
class managementClass{
    private:
        //センサーデータ
		ros::NodeHandle nhSub;
		ros::Subscriber sub;
		ros::CallbackQueue queue;
        obstacle_detection_2019::synchronizedImage syncImg;
        //送信データ
		ros::NodeHandle nhPub1,nhPub2;
        ros::Publisher pubRgb,pubDepth;
        // cv_bridge::CvImage bridgeRgb,bridgeDepth;
        sensor_msgs::Image rgbImage,depthImage;
        //
        
    public:
        //in constracter.cpp
        //コンストラクタ：クラス定義に呼び出されるメソッド
        managementClass();
        //デストラクタ：クラスが消滅するときに呼びだされるメソッド
        ~managementClass();
        //
        //メソッド：関数のようなもの:後でlaunchファイルからの読み込みメソッドを追加
        //in property.cpp
        //セット：内部パラメータの書き込み
        void setParam(int& temp);//(未使用)
        //ゲット：内部パラメータの読み込み
        int& getParam();//(未使用)
        //
        //in methods.cpp
        //その他メソッド
        //--センサーデータ受信
        void subscribeData();
        void callback(const obstacle_detection_2019::synchronizedImage::ConstPtr& imageMsg);
        //データ送信
        void publishRgbCamData();
        void publishDepthCamData();
        //デバッグ
        void delayTime();
        void culcDifTime();
};
#endif
