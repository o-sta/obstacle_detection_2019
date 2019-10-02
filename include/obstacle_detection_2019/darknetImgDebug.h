//多重インクルード防止
#ifndef INCLUDE_DARKNET_DEBUG
#define INCLUDE_DARKNET_DEBUG
#include <obstacle_detection_2019/darknetImg.h>

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
        void drawClusterCells(obstacle_detection_2019::ClassificationElement& cluster, int colorIndex);    //セルの塗りつぶし
        void setMapImageConfig();   //マップイメージの詳細設定
        void setColorMap(std::vector<int>&);
    public:
        darknetImgDebug();
        ~darknetImgDebug();
        void gridmap2PointCloud();  //グリッドマップをポイントクラウドに変換
        void gridmap2Image(obstacle_detection_2019::SensorMapDataMultiLayer& smdml, cv::Mat& image);       //グリッドマップを画像に変換
        void cluster2PointCloud();  //クラスタ情報を含むグリッドマップをポイントクラウドに変換
        void cluster2Image(obstacle_detection_2019::ClassificationData& clusterData, cv::Mat& image);       //クラスタ情報を含むグリッドマップを画像に変換
        void publishDebugData();    //パブリッシュリストに存在するデータをpublishする
};

#endif