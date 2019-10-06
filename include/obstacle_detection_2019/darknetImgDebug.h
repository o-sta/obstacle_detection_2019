//多重インクルード防止
#ifndef INCLUDE_DARKNET_DEBUG
#define INCLUDE_DARKNET_DEBUG
#include <obstacle_detection_2019/darknetImg.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>


class darknetImgDebug : public darknetImg {
    private:
        cv_bridge::CvImagePtr mapImageCB;   //クラスタ毎に色分けされたマップセル画像
        std::vector<int> colorMap;          //色付けを行うためのカラーマップ
        int cellMargin;         //セルの余白[px]
        int cellSideLength;     //セルの辺の長さ[px]
        int mapImageRows;       //マップイメージの行
        int mapImageCols;       //マップイメージの列
        std::vector<int> num_temp;  //一時変数（要素数はマップセルの数）
        void registerPubData(); //パブリッシュするデータを登録
    protected:
        /**クラスタに属しているセルを探してマップ画像に色を付ける
         * cluster 入力クラスタ
         * colorIndex 色番号
         * image 出力画像
         */
        void drawClusterCells(obstacle_detection_2019::ClassificationElement& cluster, int colorIndex, cv::Mat& image);

        /**マップ画像の詳細設定を行う
         * 画像の幅、高さをcv::Matに適用
         */ 
        void setMapImageConfig();

        /**カラーマップの配列を3の倍数(BGR?)に設定
         */ 
        void setColorMap(std::vector<int>&);
    public:
        darknetImgDebug();
        ~darknetImgDebug();

        /**グリッドマップをポイントクラウドに変換
         * 実装予定無し
         */
        void gridmap2PointCloud();

        /**グリッドマップを画像に変換
         * セルに存在する点の数が閾値よりも大きい場合、そのセルを赤色にする
         * それ以外は黒色
         * smdml マップのレイヤ(入力)
         * pt_num_threthold 閾値
         * image 出力画像
         */
        void gridmap2Image(obstacle_detection_2019::SensorMapDataMultiLayer& smdml,int pt_num_threthold ,cv::Mat& image);

        /**
         * クラスタ情報を含むグリッドマップをポイントクラウドに変換する
         */ 
        void cluster2PointCloud(obstacle_detection_2019::ClassificationData& clusterData, sensor_msgs::PointCloud2& pc_msg);
        void cluster2Image(obstacle_detection_2019::ClassificationData& clusterData, cv::Mat& image);       //クラスタ情報を含むグリッドマップを画像に変換
        void publishDebugData();    //パブリッシュリストに存在するデータをpublishする
};

#endif