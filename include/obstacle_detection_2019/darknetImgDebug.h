//多重インクルード防止
#ifndef INCLUDE_DARKNET_DEBUG
#define INCLUDE_DARKNET_DEBUG
#include <obstacle_detection_2019/darknetImg.h>

class darknetImgDebug : public darknetImg {
    private:
        void registerPubData(); //パブリッシュするデータを登録
    protected:
        void drawClusterCells(obstacle_detection_2019::ClassificationElement& cluster);    //セルの塗りつぶし
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