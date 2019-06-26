#include<obstacle_detection_2019/classification.h>

//subscribe
void classificationClass::subscribeSensorDataCamera(){//Cameraデータ受信
	queue1.callOne(ros::WallDuration(1));
}
void classificationClass::cameraMap_callback(const obstacle_detection_2019::SensorMapData::ConstPtr& msg)
{
	ROS_INFO("subscribedSensorDataCamera");
    //データをコピー
    smdCamera.header = msg->header;
    smdCamera.width = msg->width;
    smdCamera.height = msg->height;
    smdCamera.res = msg->res;
    smdCamera.widthInt = msg->widthInt;
    smdCamera.heightInt = msg->heightInt;
    smdCamera.cp = msg->cp;
    smdCamera.index = msg->index;
    smdCamera.size = msg->size;
    smdCamera.pt = msg->pt;
	//move manage method
	manage();
}
void classificationClass::subscribeSensorDataLRF(){//LRFデータ受信
	queue2.callOne(ros::WallDuration(1));
}
void classificationClass::laserMap_callback(const obstacle_detection_2019::SensorMapData::ConstPtr& msg)
{
    //データをコピー
    smdLRF.header = msg->header;
    smdLRF.width = msg->width;
    smdLRF.height = msg->height;
    smdLRF.res = msg->res;
    smdLRF.widthInt = msg->widthInt;
    smdLRF.heightInt = msg->heightInt;
    smdLRF.cp = msg->cp;
    smdLRF.index = msg->index;
    smdLRF.size = msg->size;
    smdLRF.pt = msg->pt;
	manage();
}
void classificationClass::configCallback(obstacle_detection_2019::classificationConfig &config, uint32_t level) {
	ROS_INFO("Reconfigure Request: %d %f %f %d", 
		config.windowDivisionDegree, config.windowHeight,
		config.windowWidth,config.windowMinPts
		// config.str_param.c_str(), 
		// config.bool_param?"True":"False", 
		// config.size
		);

    //窓パラメータ-->dynamic_reconfigure
    winDivDeg = config.windowDivisionDegree;
    winDivNum = (int)( (maxCamDeg - minCamDeg) / winDivDeg ) + 1;//後でチェック
    heightWin = config.windowHeight;
    widthWin = config.windowWidth;
	minPts = config.windowMinPts;
	//窓定義
	// std::vector<int> winIndex;//基準点（コア点）から参照値
	int heightWinInt = (int32_t)((float)heightWin/mapRes)*2 + 1;// heightWin / 解像度 *2 + 1
	int widthWinInt = (int32_t)((float)widthWin/mapRes)*2 + 1;// widthWin / 解像度 *2 + 1

	winIndex.resize(heightWinInt * widthWinInt);
	int count = 0;//カウント用
	for(int h = -heightWinInt/2; h <= heightWinInt/2; h++ ){
		for(int w = -widthWinInt/2; w <= widthWinInt/2; w++){
			if(h==0 && w==0 ){
				continue;
			}
			winIndex[count++] = h * mapWidthInt + w;
		}
	}
	//追加
	//launch ファイル読み出し
	// setWindowParam();
	//窓定義2(上のコードを消すまで使用変数の語尾に2を追加)
	// std::vector< std::vector<int> > winIndex2;//基準点（コア点）から参照座標
	winIndex2.resize(winDivNum);//分割個数でリサイズ
	ROS_INFO("originF--H,W:%f,%f",heightWin,widthWin);
	ROS_INFO("originI--H,W:%d,%d",heightWinInt,widthWinInt);
	//各窓ごとに定義
	for(int k=0; k<winIndex2.size(); k++){
		int deg = minCamDeg + winDivDeg/2 + winDivDeg * k;
		//探索範囲の算出
		//窓の傾き角度(センサ正面を0度, 反時計回りを正)
		float theta = (float)(deg)/180.0*M_PI;
		// float theta = (float)(-deg)/180.0*M_PI;
		float thetaAbs = std::abs(theta);
		//探索サイズ
		// float searchRangeH = widthWin* cos(thetaAbs) + heightWin*sin(thetaAbs);
		// float searchRangeW = widthWin* sin(thetaAbs) + heightWin*cos(thetaAbs);
		float searchRangeW = widthWin* cos(thetaAbs) + heightWin*sin(thetaAbs);
		float searchRangeH = widthWin* sin(thetaAbs) + heightWin*cos(thetaAbs);
		//セル数に変換
		int searchRangeHInt = (int32_t)((float)searchRangeH/mapRes)*2 + 1;// heightWin / 解像度 *2 + 1
		int searchRangeWInt = (int32_t)((float)searchRangeW/mapRes)*2 + 1;// widthWin / 解像度 *2 + 1
		//リサイズ用カウンタ
		int count2 = 0;
		winIndex2[k].resize(searchRangeHInt*searchRangeWInt);

        // if(k == (int)(winIndex2.size()/2)){
		ROS_INFO("deg,H,W:%d,%d,%d",deg,searchRangeHInt,searchRangeWInt);
        // }
		for(int h = -searchRangeHInt/2; h <= searchRangeHInt/2; h++ ){
			for(int w = -searchRangeWInt/2; w <= searchRangeWInt/2; w++){
				//h,wは, すでに探索セル(探索窓中心座標)からの座標差を示している
				//回転行列
				float dw = w*cos(theta) + h*sin(theta);
				float dh = -w*sin(theta) + h*cos(theta);
				// ROS_INFO("H,W:%d,%d",(int)dh,(int)dw);
				// ROS_INFO("ifH,ifW:%d,%d",std::abs((int)dh) <= heightWinInt/2.0,std::abs((int)dw) <= widthWinInt/2.0);
				//座標が窓内に存在するか
				if(std::abs((int)dw) <= widthWinInt/2.0 && std::abs((int)dh) <= heightWinInt/2.0){
					//探索インデックスに追加
					// ROS_INFO("add H,W:%d,%d",(int)dh,(int)dw);
					winIndex2[k][count2++] = h * mapWidthInt + w;
					// winIndex2[k][count2++] = (int)dh * mapWidthInt + (int)dw;
				}
			}
		}
		winIndex2[k].resize(count2);
	}
	showSearchWindows();
	showSearchWindows((int)config.debugAngle+minCamDeg);
}
void classificationClass::manage(){
	//
	ROS_INFO("classificationDBSCAN");
	classificationDBSCAN();
	if(!isClassificationData()){
		ROS_INFO("No Classification data");
	}
	ROS_INFO("publishClassificationData");
	publishClassificationData();
	ROS_INFO("showCluster");
	showCluster();
	ROS_INFO("clearMessages");
	clearMessages();
}
void classificationClass::classificationDBSCAN(){//カメラ

	// 送信データ作成
	// clustedData cd;
	cd.header = smdCamera.header;
	//map設定データ
	cd.width = smdCamera.width;
	cd.height = smdCamera.height;
	cd.res = smdCamera.res;
	cd.widthInt = smdCamera.widthInt;
	cd.heightInt = smdCamera.heightInt;
	cd.cp = smdCamera.cp;
	cd.size.data = 0;
	//障害物データ数確保
	cd.data.resize(smdCamera.index.size());

	//mapIndexデータ
	// mapIndex = -1 -> searched or not include obstacle point
	std::vector<int> mapIndex;
	// mapIndex.insert(mapIndex.end(), smdCamera.index.begin(), smdCamera.index.end());
	mapIndex.resize(smdCamera.index.size());
	for(int k=0; k < mapIndex.size(); k++){
		mapIndex[k] = smdCamera.index[k].data;
	}
	// DBSCAN開始
	for(int k = 0; k < mapIndex.size(); k++){//k : 0 -> widthInt * heightInt -1 
		//マップセルにデータがない場合：スキップ
		if(mapIndex[k] < 0){
			continue;
		}
		// データがある場合
		//探索済み点群を反映したIndex : taskIndexのfor文中での重複探査を防止
		std::vector<int> searchedIndex;
		searchedIndex = mapIndex;
		// クラスタデータに追加
		// クラスタサイズ拡張
		cd.size.data += 1;
		int clusterNum = cd.size.data - 1;
		// リサイズ
		cd.data[clusterNum].index.resize((int)mapIndex.size() - k);//残り最大サイズ
		cd.data[clusterNum].pt.resize((int)mapIndex.size() - k);//残り最大サイズ
		// データ代入(初期化)
		cd.data[clusterNum].size.data = 0;
		cd.data[clusterNum].dens.data = 0;
		cd.data[clusterNum].gc.x = 0;
		cd.data[clusterNum].gc.y = 0;
		cd.data[clusterNum].gc.z = 0;
		
		//タスクデータ作成
		std::vector<int> taskIndex;//候補点群の格納
		int taskSize = 0;//taskIndexのサイズ
		taskIndex.resize((int)mapIndex.size() - k);//残り最大サイズ
		taskIndex[taskSize++] = k;

		//タスク追加データを重複探査防止 
		searchedIndex[k] = -1;

		for(int n=0; n < taskSize; n++){
			int count = 0;//密度（点の数）
			std::vector<int> tempIndex;//Window探索中のみ使用するIndex
			tempIndex.resize(winIndex.size());
			int tempSize = 0;
			//窓内を探索
			for(int m=0; m < winIndex.size(); m++){
				int pos = taskIndex[n] + winIndex[m];
				int posW = taskIndex[n] % smdCamera.widthInt.data;
				int moveW = winIndex[m] % smdCamera.widthInt.data;
				//ここの処理, 後で確認
				int difW = posW + moveW;
				if(pos < 0 || pos >= (int)mapIndex.size()
					|| difW < 0 || difW > smdCamera.widthInt.data ){
						//マップ範囲外検索
						continue;
				}
				//セルにデータがない場合：スキップ
				if(mapIndex[pos] < 0){
					continue;
				}
				//点の数（密度）を追加
				count += smdCamera.size[ mapIndex[pos] ].data;

				//探査済みの場合：スキップ
				if(searchedIndex[pos]< 0){
					continue;
				}
				//次のタスク候補点（コア点候補）として追加
				tempIndex[tempSize++] = pos;
			}
			//リサイズ
			tempIndex.resize(tempSize);

			//点の数（密度）を評価
			// (smdCamera.height/2+smdCamera.cp) : センサからマップの上端までの距離
			// taskIndex[n] / smdCamera.widthInt * smdCamera.res : マップ上端から障害物セルまでの距離
			float y=(smdCamera.height.data/2.0+smdCamera.cp.y) - taskIndex[n] / smdCamera.widthInt.data * smdCamera.res.data;
			//奥行yに対する評価式らしい
			// int minPts=(int)( -35*y*y+1200 );
			// int minPts=10;
			//評価式よりカウントが小さい時: スキップ
			if(count < minPts){
				continue;
			}
			//コア点をクラスタに追加
			// データ代入
			//mapIndex[k] <<< k=taskIndex[n] 
			cd.data[clusterNum].size.data += 1;
			cd.data[clusterNum].dens.data += smdCamera.size[ mapIndex[taskIndex[n]] ].data;
			cd.data[clusterNum].gc.x += smdCamera.pt[ mapIndex[taskIndex[n]] ].x*smdCamera.size[ mapIndex[taskIndex[n]] ].data ;
			cd.data[clusterNum].gc.y += smdCamera.pt[ mapIndex[taskIndex[n]] ].y*smdCamera.size[ mapIndex[taskIndex[n]] ].data ;
			cd.data[clusterNum].gc.z += smdCamera.pt[ mapIndex[taskIndex[n]] ].z*smdCamera.size[ mapIndex[taskIndex[n]] ].data ;
			cd.data[clusterNum].index[cd.data[clusterNum].size.data - 1].data = taskIndex[n];//マップセルに対するインデックス, 1データ番号->マップセル
			cd.data[clusterNum].pt[cd.data[clusterNum].size.data - 1] = smdCamera.pt[ mapIndex[taskIndex[n]] ];
			//重複探査防止
			// tempIndex（タスク候補点をタスクに追加）
			for(int m=0; m < tempIndex.size(); m++){
				//追加
				taskIndex[taskSize++] = tempIndex[m];
				//重複探査防止
				searchedIndex[tempIndex[m]] = -1;
			}
		}
		if(cd.data[clusterNum].size.data==0){
			mapIndex[cd.data[clusterNum].index[0].data] = -1;
			cd.size.data -=1;
			continue;
		}
		//リサイズ
		cd.data[clusterNum].index.resize(cd.data[clusterNum].size.data);
		//追加したデータをマップから削除
		for(int n=0; n < cd.data[clusterNum].index.size(); n++){
			//mapIndex[データ番号[n]->マップ位置] = データ無し
			mapIndex[cd.data[clusterNum].index[n].data] = -1;
		}
			
		//タスクデータ探査終了
		//平均点算出
		cd.data[clusterNum].gc.x /= cd.data[clusterNum].dens.data;
		cd.data[clusterNum].gc.y /= cd.data[clusterNum].dens.data;
		cd.data[clusterNum].gc.z /= cd.data[clusterNum].dens.data;
		//リサイズ
		cd.data[clusterNum].pt.resize(cd.data[clusterNum].size.data);
	}
	// DBSCAN終了
	//データリサイズ
	cd.data.resize(cd.size.data);
}
//追加
//使用する窓を選択
//入力:角度(deg)
//反時計回り, y軸を0度と置く
//出力:使用する窓番号
int classificationClass::selectWindow(int& angle){
	int selectNum = winDivNum / 2;//カメラ正面方向の窓番号
	if(std::abs(angle) > winDivDeg/2){//正面方向ではない
		//angle / winDivDeg
		//第一象限ではマイナス
		//第二象限ではプラス
		//selectNumは正面の番号
		selectNum = selectNum + (angle / winDivDeg);
	}
	return selectNum;
}

void classificationClass::publishClassificationData(){//データ送信
    pub.publish(cd);
}
void classificationClass::clearMessages(){
    smdCamera.index.clear();
    smdCamera.pt.clear();
    smdCamera.size.clear();
    smdLRF.index.clear();
    smdLRF.pt.clear();
	cd.data.clear();
	// compCamData.index.clear();
	// compCamData.pt.clear();
	// compCamData.size.clear();
}