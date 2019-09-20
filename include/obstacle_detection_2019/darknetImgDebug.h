#include <obstacle_detection_2019/darknetImg.h>

class darknetImgDebug : public darknetImg {
    private:
        void registerPubData(); //パブリッシュするデータを登録
    public:
        darknetImgDebug();
        ~darknetImgDebug();
        void gridmap2PointCloud();  //グリッドマップをポイントクラウドに変換
        void gridmap2Image();       //グリッドマップを画像に変換
        void cluster2PointCloud();  //クラスタ情報を含むグリッドマップをポイントクラウドに変換
        void cluster2Image();       //クラスタ情報を含むグリッドマップを画像に変換
        void publishDebugData();    //パブリッシュリストに存在するデータをpublishする
};