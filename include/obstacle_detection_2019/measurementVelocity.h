//多重インクルード防止
#ifndef INCLUDE_MEASUREMENT_VELOCITY_CLASS
#define INCLUDE_MEASUREMENT_VELOCITY_CLASS
//include haeders
#include <ros/ros.h>
#include <ros/callback_queue.h>
//self msg
#include <obstacle_detection_2019/SensorMapData.h>
#include <obstacle_detection_2019/CompressedSensorData.h>
#include <obstacle_detection_2019/ClassificationVelocityData.h>
#include <obstacle_detection_2019/ClassificationData.h>
#include <obstacle_detection_2019/ImageMatchingData.h>

//クラスの定義
class measurementVelocity{
    private:
        //受信データ
		ros::NodeHandle nhSub1,nhSub2;
		ros::Subscriber subCluster,subMatching;
		ros::CallbackQueue queue1,queue2;
        obstacle_detection_2019::ClassificationData curClstr,prvClstr;
        obstacle_detection_2019::ImageMatchingData matchData;
        //送信データ
		ros::NodeHandle nhPub;
        ros::Publisher pub;
    	obstacle_detection_2019::ClassificationVelocityData cvd;//速度データ付きのクラスタデータ
        //クラスタマップ
    	std::vector<int> prvClstrMap, curClstrMap;
    	//マッチング評価
	    std::vector<int> matchResult;
    public:
        //in constracter.cpp
        //コンストラクタ：クラス定義に呼び出されるメソッド
        measurementVelocity();
        //デストラクタ：クラスが消滅するときに呼びだされるメソッド
        ~measurementVelocity();
        //
        //メソッド:後でlaunchファイルからの読み込みメソッドを追加
        //in property.cpp
        //セット：内部パラメータの書き込み
        void setParam(int& temp);//(未使用)
        //ゲット：内部パラメータの読み込み
        int& getParam();//(未使用)
        //
        //in methods.cpp
        //その他メソッド
        //--センサーデータ受信
        void subscribeClusterData();//データ受信
        void cluster_callback(const obstacle_detection_2019::SensorMapData::ConstPtr& msg);
        void subscribeImageMatchingData();//データ受信
        void matching_callback(const obstacle_detection_2019::SensorMapData::ConstPtr& msg);
        //--マッチング
        bool isPrvClstr();//連続データがあるか確認
        void creatClstrMap();//クラスタマップ作製
        void matchingClstr();//マッチング処理（途中）
        void measurementProcess();//計測処理
        // クラスタデータ送信
        void publishClassificationData();//データ送信
        //データ更新
        void renewClstrData();
};
#endif