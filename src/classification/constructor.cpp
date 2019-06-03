#include<obstacle_detection_2019/classification.h>

classificationClass::classificationClass()
{
	//subscriber
	nhSub1.setCallbackQueue(&queue1);
	subCam=nhSub1.subscribe("/cam_test",1,&classificationClass::cameraMap_callback,this);
	nhSub2.setCallbackQueue(&queue2);
	subLRF=nhSub2.subscribe("/",1,&classificationClass::laserMap_callback,this);
	//publisher
    pub= nhPub.advertise<obstacle_detection_2019::ClassificationData>("classificationData", 1);

	//窓定義
	// std::vector<int> winIndex;//基準点（コア点）から参照値
	float heightWin = 0.1;// 0.1[m] 
	float widthWin = 0.4;// 0.4[m] 
	int heightWinInt = (int32_t)((float)heightWin/smdCamera.res.data)*2 + 1;// heightWin / 解像度 *2 + 1
	int widthWinInt = (int32_t)((float)widthWin/smdCamera.res.data)*2 + 1;// widthWin / 解像度 *2 + 1

	winIndex.resize(heightWinInt * widthWinInt);
	int count = 0;//カウント用
	for(int h = -heightWinInt/2; h <= heightWinInt/2; h++ ){
		for(int w = -widthWinInt/2; w <= widthWinInt/2; w++){
			if(h==0 && w==0 ){
				continue;
			}
			winIndex[count++] = h * smdCamera.widthInt.data + w;
		}
	}

	
}
classificationClass::~classificationClass(){
	winIndex.clear();
}