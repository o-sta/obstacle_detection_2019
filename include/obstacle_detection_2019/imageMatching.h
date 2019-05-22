//多重インクルード防止
#ifndef INCLUDE_IMAGE_MATCHING_CLASS
#define INCLUDE_IMAGE_MATCHING_CLASS
//include haeders
#include <ros/ros.h>
#include <ros/callback_queue.h>
//for image processing on ros
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>//オプティカルフロー用
#include <opencv2/features2d/features2d.hpp>
//self msg
#include <obstacle_detection_2019/MaskImageData.h>
#include <obstacle_detection_2019/ImageMatchingData.h>

//クラスの定義
class imageMatchingClass{
    private:
        //センサーデータ
		ros::NodeHandle nhSub1,nhSub2;
		ros::Subscriber subImg,subMskImg;
		ros::CallbackQueue queue1,queue2;
        cv_bridge::CvImagePtr bridgeImagePre,bridgeImageCur;
        obstacle_detection_2019::MaskImageData midPre,midCur;
        //送信データ
		ros::NodeHandle nhPub;
        ros::Publisher pubMatch;
        obstacle_detection_2019::ImageMatchingData matchData;
        //マップパラメータ
        float mapW;//width[m]
        float mapH;//height[m]
        float mapR;//resolution[m]
        int mapWi;//マップサイズWidth[pixel]
        int mapHi;//マップサイズHeight[pixel]
        cv::Point2f posCamera;//カメラの位置(2次元マップ上)(未使用)
        //特徴点抽出
        cv::Mat grayImgPre,grayImgCur;//グレースケール
        int nh,nw;//画像分割個数
        int maxDetectPoint;//最大検出数
        int maxPoint;//最大抽出数
        cv::Mat clipImg;//分割画像Mat
        //特徴点マッチング
        std::vector< cv::Point2f > featurePointsPre;
        std::vector< cv::Point2f > featurePointsCur;
        std::vector< cv::Point2f > featurePointsTemp;
		std::vector<uchar> sts;
		std::vector<float> ers;
		int ws;//window size
    public:
        //in constracter.cpp
        //コンストラクタ：クラス定義に呼び出されるメソッド
        imageMatchingClass();
        //デストラクタ：クラスが消滅するときに呼びだされるメソッド
        ~imageMatchingClass();
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
        void subscribeImageData();//データ受信(RGB画像)
        void subscribeMaskImageData();//データ受信(mask画像)
        void image_callback(const sensor_msgs::ImageConstPtr& msg);
        void maskImage_callback(const obstacle_detection_2019::MaskImageData::ConstPtr& msg);
        //--グレースケール化
        void cvtGray();
        //--データ確認
        bool isBridgeImagePre();//
        //データ更新
        void resetData();
        //特徴点抽出
        void getFeaturePoints();//特徴点抽出
        void featureMatching();//特徴点マッチング
        void checkMatchingError();//エラーチェック
        void creatMatchingData();//
        //--座標変換
        //センサ座標系ー＞マップ座標系
        bool convertToGrid(const float& x,const float& y,int& xg,int& yg);
        //センサデータ送信
        void publishMatchingData();//データ送信
};
#endif