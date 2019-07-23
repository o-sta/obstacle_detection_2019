#include <ros/ros.h>
#include <obstacle_detection_2019/imageMatching.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"obstacle_detection_2019_im");
	
    imageMatchingClass imc; //image matching 
	while(ros::ok())
	{	
		//subscribe
		ROS_INFO("subscribeImageData");
		imc.subscribeImageData();
		ROS_INFO("subscribeMaskImageData");
		imc.subscribeMaskImageData();
		// ROS_INFO("if(imc.isBridgeImagePre() && imc.isMaskImagePre())=%d,%d",
		// 	imc.isBridgeImagePre(), imc.isMaskImagePre());
		if(imc.isBridgeImageCur()){
			ROS_INFO("cvtGray");
			imc.cvtGray();
			//特徴点抽出
			ROS_INFO("getFeaturePoints");
			imc.getFeaturePoints();
		}
		//データが２つ以上ある時にマッチング処理をする
		if(imc.isBridgeImagePre() && imc.isMaskImagePre() && imc.isFpointPre()){
			//特徴点マッチング
			ROS_INFO("featureMatching");
			imc.featureMatching();
			ROS_INFO("checkMatchingError");
			//マッチングミスをデータから除去
			imc.checkMatchingError();
			ROS_INFO("creatMatchingData");
			//publishデータ生成
			imc.creatMatchingData();
			ROS_INFO("publishMatchingData");
			//publish
			imc.publishMatchingData();
			// imc.showMatchingMap();
			imc.showMatchingImage();
		}
		ROS_INFO("resetData");
		//データ更新
		imc.resetData();
	}
    return 0;
}