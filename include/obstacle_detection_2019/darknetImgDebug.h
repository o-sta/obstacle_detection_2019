//多重インクルード防止
#ifndef INCLUDE_DARKNET_DEBUG
#define INCLUDE_DARKNET_DEBUG
#include <obstacle_detection_2019/darknetImg.h>

class darknetImgDebug : public darknetImg {
    private:
        cv_bridge::CvImagePtr mapImageCB; //クラスタ毎に色分けされたマップセル画像
        cv::Mat colorMap;       //色付けを行うためのカラーマップ
        int cellMargin;         //セルの余白[px]
        int cellSideLength;     //セルの辺の長さ[px]
        int mapImageRows;       //マップイメージの行
        int mapImageCols;       //マップイメージの列
        void registerPubData(); //パブリッシュするデータを登録
    protected:
        void drawClusterCells(obstacle_detection_2019::ClassificationElement& cluster, int colorMode);    //セルの塗りつぶし
        void setMapImageConfig();   //マップイメージの詳細設定
        void setColorMap();
    public:
        darknetImgDebug();
        ~darknetImgDebug();
        void gridmap2PointCloud();  //グリッドマップをポイントクラウドに変換
        void gridmap2Image();       //グリッドマップを画像に変換
        void cluster2PointCloud();  //クラスタ情報を含むグリッドマップをポイントクラウドに変換
        void cluster2Image();       //クラスタ情報を含むグリッドマップを画像に変換
        void publishDebugData();    //パブリッシュリストに存在するデータをpublishする
};

#endif