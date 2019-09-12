//多重インクルード防止
#ifndef INCLUDE_DETECT_PERSON_HOG
#define INCLUDE_DETECT_PERSON_HOG
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
        cv_bridge::CvImagePtr bridgeImage;
        darknet_ros_msgs::BoundingBoxes boundingBoxesMsg;
        //メッセージフィルタ
        message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bb_sub;
        message_filters::Subscriber<sensor_msgs::Image> image_sub;
        typedef message_filters::sync_policies::ApproximateTime<darknet_ros_msgs::BoundingBoxes, sensor_msgs::Image> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync;
        //パラメータ設定（後でlaunchに移動）
        float camHeight = 30.0; //適当
        float groundCandidateY = 26.4; //適当
        float f = 600.35; //適当
        float ground_th;
        float a,b,c,d;
        float height_th;
        int minPts;
        //マップのパラメータ
        float mapWidth, mapHeight, mapResolution;
        int mapRows, mapCols, numberOfCells;
        //窓
        std::vector<int> cellsInWindow;
        //関数用
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        pcl::PointIndices::Ptr inliers;
        pcl::ModelCoefficients::Ptr coefficients;
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points;
        obstacle_detection_2019::SensorMapDataMultiLayer smdml;
        obstacle_detection_2019::SensorMapDataMultiLayer smdmlLowDimension;
        obstacle_detection_2019::ClassificationData cd;
        //深度画像のマスク（有効で1、無効で0）
        //std::vector<std::vector<char>> mask;
        cv::Mat mask;
        bool is_size_initialized; //画像サイズが初期化されたか
    protected:
        bool convertToGrid(const float& x,const float& y,int& xg,int& yg);
        //窓内に含まれているセルを出力する関数
    /* data */
    public:
        darknetImg(/* args */);
        ~darknetImg();
        void detect(); //人間を検出
        bool subscribeSensorData();//データ受信
        void sensor_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bb,const sensor_msgs::ImageConstPtr& image); //データ受信
        void publishData(); //データ送信
        //床面除去関数
        void pickUpGroundPointCandidates(); //床面候補点の選択
        void estimateGroundCoefficients(); //床面係数abcdの算出
        void removeGroundPoints(); //床面の点を除外する
        //歩行者位置推定関数
        void trimPoints(); //枠線を元に点をトリミング（枠外の点を除去）
        void generateGridmap(); //深度画像からグリッドマップ作成
        void dimensionalityReductionGridmap(); // グリッドマップのを行のみ（奥行きのみ）のマップに変換
        void classifyPoints(); //DBSCANによるクラスタリング
        void estimatePersonPosition(); //歩行者位置推定
        void predictPersonPosition(); //歩行者位置予測（カルマンフィルタ）※未実装
};

#endif

