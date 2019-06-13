#include<obstacle_detection_2019/classification.h>

classificationClass::classificationClass()
	:minCamDeg(-45),maxCamDeg(45),winDivDeg(10),winDivNum(9)
{
	//subscriber
	nhSub1.setCallbackQueue(&queue1);
	subCam=nhSub1.subscribe("/cameraMapData",1,&classificationClass::cameraMap_callback,this);
	nhSub2.setCallbackQueue(&queue2);
	subLRF=nhSub2.subscribe("/lrf",1,&classificationClass::laserMap_callback,this);
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
	//追加
	//launch ファイル読み出し
	setWindowParam();
	//窓定義2(上のコードを消すまで使用変数の語尾に2を追加)
	// std::vector< std::vector<int> > winIndex2;//基準点（コア点）から参照座標
	winIndex2.resize(winDivNum);//分割個数でリサイズ
	//実測窓サイズ
	float heightWin2 = 0.1;// 0.1[m] 
	float widthWin2 = 0.4;// 0.4[m] 
	int heightWinInt2 = (int32_t)((float)heightWin2/smdCamera.res.data)*2 + 1;// heightWin / 解像度 *2 + 1
	int widthWinInt2 = (int32_t)((float)widthWin2/smdCamera.res.data)*2 + 1;// widthWin / 解像度 *2 + 1

	//各窓ごとに定義
	for(int k=0; k<winIndex2.size(); k++){
		int deg = minCamDeg + winDivDeg/2 + winDivDeg * k;
		//探索範囲の算出
		//窓の傾き角度(センサ正面を0度, 反時計回りを正)
		float theta = (float)(deg)/180.0*M_PI;
		float thetaAbs = std::abs(theta);
		//探索サイズ
		float searchRangeH = widthWin2* cos(thetaAbs) + heightWin2*sin(thetaAbs);
		float searchRangeW = widthWin2* sin(thetaAbs) + heightWin2*cos(thetaAbs);
		//セル数に変換
		int searchRangeHInt = (int32_t)((float)searchRangeH/smdCamera.res.data)*2 + 1;// heightWin / 解像度 *2 + 1
		int searchRangeWInt = (int32_t)((float)searchRangeW/smdCamera.res.data)*2 + 1;// widthWin / 解像度 *2 + 1
		//リサイズ用カウンタ
		int count2 = 0;
		winIndex2[k].resize(searchRangeHInt*searchRangeWInt);
		for(int h = -searchRangeHInt/2; h <= searchRangeHInt/2; h++ ){
			for(int w = -searchRangeWInt/2; w <= searchRangeWInt/2; w++){
				//h,wは, すでに探索セル(探索窓中心座標)からの座標差を示している
				//回転行列
				float dw = w*cos(theta) + h*sin(theta);
				float dh = -w*sin(theta) + h*cos(theta);
				//座標が窓内に存在するか
				if(std::abs(dw) < widthWinInt2/2.0 && std::abs(dh) < heightWinInt2/2.0){
					//探索インデックスに追加
					winIndex2[k][count2++] = h * smdCamera.widthInt.data + w;
				}
			}
		}
		winIndex2[k].resize(count2);
	}
	//デバッグ用
	//publisher
    pubDeb= nhDeb.advertise<sensor_msgs::Image>("windowImage", 1);
    pubDebPcl= nhDebPcl.advertise<sensor_msgs::PointCloud2>("visualizedCluster", 1);
}
classificationClass::~classificationClass(){
	winIndex.clear();
}