//多重インクルード防止
#ifndef INCLUDE_CLASSIFICATION_CLASS
#define INCLUDE_CLASSIFICATION_CLASS
//include haeders
#include <ros/ros.h>
#include <ros/callback_queue.h>
//self msg
#include <obstacle_detection_2019/SensorMapData.h>
#include <obstacle_detection_2019/CompressedSensorData.h>
#include <obstacle_detection_2019/ClassificationData.h>
//追加（デバッグ用）
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl_ros/point_cloud.h>

//クラスの定義
class classificationClass{
    private:
        //受信データ
		ros::NodeHandle nhSub1, nhSub2;
		ros::Subscriber subCam, subLRF;
		ros::CallbackQueue queue1, queue2;
        obstacle_detection_2019::SensorMapData smdCamera, smdLRF;
        //送信データ
		ros::NodeHandle nhPub;
        ros::Publisher pub;
        obstacle_detection_2019::ClassificationData cd;
        //クラスタリング処理
        std::vector<int> mapIndex;//
    	std::vector<int> winIndex;//基準点（コア点）から参照値
        obstacle_detection_2019::CompressedSensorData compCamData;
        //--追加
        int minCamDeg, maxCamDeg;
        int winDivDeg;
        int winDivNum;//(int)( ステレオカメラの視野角の半分(45度) / winDivDeg ) * 2 + 1
        std::vector< std::vector<int> > winIndex2;
        //----デバッグ用
		ros::NodeHandle nhDeb,nhDebPcl;
        ros::Publisher pubDeb,pubDebPcl;
        //--
    public:
        //in constracter.cpp
        //コンストラクタ：クラス定義に呼び出されるメソッド
        classificationClass();
        //デストラクタ：クラスが消滅するときに呼びだされるメソッド
        ~classificationClass();
        //
        //メソッド:後でlaunchファイルからの読み込みメソッドを追加
        //in property.cpp
        //セット：内部パラメータの書き込み
        void setWindowParam();//launchファイルからの探索窓パラメータ書き込み
        //ゲット：内部パラメータの読み込み
        int& getParam();//(未使用)
        //
        //in methods.cpp
        //その他メソッド
        //--センサーデータ受信
        void subscribeSensorDataCamera();//データ受信
        void cameraMap_callback(const obstacle_detection_2019::SensorMapData::ConstPtr& msg);
        void subscribeSensorDataLRF();//データ受信
        void laserMap_callback(const obstacle_detection_2019::SensorMapData::ConstPtr& msg);
        //--クラスタリング
        void sortSensorData();//カメラデータソート
        void compressSensorData();//データ圧縮
        void creatMapIndex();
        void classificationDBSCAN();//DBSCAN
        //追加  
        int selectWindow(int& angle);//窓選択

        // クラスタデータ送信
        void publishClassificationData();//データ送信
        // データクリア
        void clearMessages();
        //追加  
        void showSearchWindows();
        void showCluster();
};
#endif