#include <ros/ros.h>
#include <obstacle_detection_2019/imageMatching.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"obstacle_detection_2019_im");
	
    imageMatchingClass imc; //image matching 
	while(ros::ok())
	{	
		//subscribe
		imc.subscribeImageData();
		imc.subscribeMaskImageData();
		//データが２つ以上ある時にマッチング処理をする
		if(imc.isBridgeImagePre() && imc.isMaskImagePre()){
			//特徴点抽出
			imc.getFeaturePoints();
			//特徴点マッチング
			imc.featureMatching();
			//マッチングミスをデータから除去
			imc.checkMatchingError();
			//publishデータ生成
			imc.creatMatchingData();
			//publish
			imc.publishMatchingData();
		}
		//データ更新
		imc.resetData();
	}
    return 0;
}