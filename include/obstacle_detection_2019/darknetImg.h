//多重インクルード防止
#ifndef INCLUDE_DARKNET
#define INCLUDE_DARKNET
//include haeders
#include <ros/ros.h>
#include <ros/callback_queue.h>
//for image processing on ros
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//PointCloud
#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
//OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/opencv.hpp>
//メッセージインクルード
//darknet_ros_msgs
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
//obstacle_detection_2019_msgs
#include <obstacle_detection_2019/SensorMapDataMultiLayer.h>
#include <obstacle_detection_2019/ClassificationData.h>
//geometry_msgs
#include <geometry_msgs/Point.h>
//メッセージフィルタ
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//Dynamic_reconfigure
#include <dynamic_reconfigure/server.h>
#include <obstacle_detection_2019/darknetImgConfig.h>

//shared ptr
//#include <boost/shared_ptr.hpp>
//#include <boost/make_shared.hpp>

//※将来的にはパブリッシャとサブスクライバを用いた入力と出力を行うクラスと
//  処理を行うクラスに分ける予定（一緒だと処理の流れが追いにくく、改変が難しい）
class darknetImg {
    private:
        //ノードハンドルとサブスクライパ、パブリッシャ
		ros::NodeHandle nhSub;
		ros::Subscriber sub;
		ros::NodeHandle nhPub;
        ros::Publisher pub;
        //センサーデータ
        cv_bridge::CvImagePtr bridgeImage;                  //深度画像
        darknet_ros_msgs::BoundingBoxes boundingBoxesMsg;   //検出された人物の枠線
        //メッセージフィルタ
        message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bb_sub;
        message_filters::Subscriber<sensor_msgs::Image> image_sub;
        typedef message_filters::sync_policies::ApproximateTime<darknet_ros_msgs::BoundingBoxes, sensor_msgs::Image> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync;
        //パラメータ設定（後でlaunchに移動）
        float camHeight = 30.0;         //床からカメラまでの高さ（値は適当）
        float groundCandidateY = 26.4;  //（値は適当）
        float f = 600.35;               //カメラの焦点距離と画像の関連パラメータ（値は適当）
        float ground_th;                //床面候補点の高さの閾値（値は適当）
        float a,b,c,d;                  //床面方程式の係数
        int minPts;                     //窓内に含まれる点の数の閾値
        int ransacNum;                  //RANSACを行う回数
        float distanceThreshold;        //床面モデルとどのくらい離れてもいいか
        float epsAngle;                 //許容できる平面
        int windowRangeCell;
        //窓
        std::vector<int> cellsInWindow;             //ウィンドウ内に入っているセルの数(セル番号を格納する)
        //関数用
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;  //ポイントクラウド
        pcl::PointIndices::Ptr inliers;             //RANSAC後の点群データ
        pcl::ModelCoefficients::Ptr coefficients;   //床面方程式の係数
        pcl::SACSegmentation<pcl::PointXYZ> seg;    //セグメンテーション
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points;                  //床面候補点
        obstacle_detection_2019::SensorMapDataMultiLayer smdml;             //複数マップ
        obstacle_detection_2019::SensorMapDataMultiLayer smdmlLowDimension; //複数マップ(cols=1)
        //--rqt_reconfigure
        dynamic_reconfigure::Server<obstacle_detection_2019::darknetImgConfig> server;
        dynamic_reconfigure::Server<obstacle_detection_2019::darknetImgConfig>::CallbackType fc;
        //深度画像のマスク（有効で1、無効で0）
        //std::vector<std::vector<char>> mask;
        cv::Mat mask;
        bool is_size_initialized; //画像サイズが初期化されたか
    protected:
        //ローカルマップのパラメータ
        float mapWidth, mapHeight, mapResolution;   //ローカルマップの横幅[m]、縦幅[m]、解像度[m/pixel]
        int mapRows, mapCols, numberOfCells;        //ローカルマップの横幅[pixel]、縦幅[pixel]、マップのセル数[pixel]
        obstacle_detection_2019::ClassificationData cd;                     //クラスタデータ
        bool convertToGrid(const float& x,const float& y,int& xg,int& yg);
        //窓内に含まれているセルを出力する関数
    /* data */
    public:
        darknetImg();
        ~darknetImg();
        void detect();              //人間を検出
        void sensor_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bb,const sensor_msgs::ImageConstPtr& image); //データ受信
        void configCallback(obstacle_detection_2019::darknetImgConfig &config, uint32_t level); //パラメータ受信
        void setLaunchParam();
        bool subscribeSensorData(); //データ受信
        void publishData();         //データ送信
        //パラメータ設定関数
        void createWindow();        //窓作成
        //床面除去関数
        void pickUpGroundPointCandidates();     //床面候補点の選択
        void estimateGroundCoefficients();      //床面係数abcdの算出
        void removeGroundPoints();              //床面の点を除外する
        //歩行者位置推定関数
        void trimPoints();                      //枠線を元に点をトリミング（枠外の点を除去）
        void generateGridmap();                 //深度画像からグリッドマップ作成
        void dimensionalityReductionGridmap();  // グリッドマップのを行のみ（奥行きのみ）のマップに変換
        void classifyPoints();                  //DBSCANによるクラスタリング
        void estimatePersonPosition();          //歩行者位置推定
        void predictPersonPosition();           //歩行者位置予測（カルマンフィルタ）※未実装
};

#endif

