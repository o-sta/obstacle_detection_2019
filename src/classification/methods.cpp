#include<obstacle_detection_2019/classification.h>

//subscribe
void classificationClass::subscribeSensorDataCamera(){//Cameraデータ受信
	queue1.callOne(ros::WallDuration(1));
}
void classificationClass::cameraMap_callback(const obstacle_detection_2019::SensorMapData::ConstPtr& msg)
{
    //データをコピー
    smdCamera.header = msg->header;
    smdCamera.width = msg->width;
    smdCamera.height = msg->height;
    smdCamera.res = msg->res;
    smdCamera.widthInt = msg->widthInt;
    smdCamera.heightInt = msg->heightInt;
    smdCamera.cp = msg->cp;
    smdCamera.index = msg->index;
    smdCamera.pt = msg->pt;
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
    smdLRF.pt = msg->pt;
}
void classificationClass::sortSensorData()
{	
	// ソート内容
	// index[j]の大きさによってソート
	// 
	// カメラデータ
	// データサイズチェック
	if(smdCamera.index.size()!=smdCamera.pt.size()){
		ROS_ERROR("MessageSize error : (index,pt) = (%d,%d)", (int)smdCamera.index.size(), (int)smdCamera.pt.size());
	}
	// ソート(バブルソート)
	for(int i = 0; i < ((int)smdCamera.index.size() - 1); i++){
		std::cout<<"i:"<<i<<std::endl;
		for(int j= ((int)smdCamera.index.size() - 1); j > i; j--){
			if(smdCamera.index[j].data < smdCamera.index[j - 1].data){
				//index, pt の組 を入れ替えていく
				 std::iter_swap(smdCamera.index.begin()+j, smdCamera.index.begin()+ j-1);
				 std::iter_swap(smdCamera.pt.begin()+j, smdCamera.pt.begin()+ j-1);
			}
		}
	}
	// sort(fruits.begin(), fruits.end(), 
	// 	[](const fruit& x, const fruit& y) { return x.name < y.name;});
}
//データ前処理(カメラデータ)
void classificationClass::compressSensorData()
{
	// sensorData
	// index : センサデータ番号k -> マップ位置 index[i]
	// index[i] 
	// -> マップ位置(w,h) : 
	// w=index[i] % widthInt
	// h=index[i] / heightInt 

	// 圧縮後データ 初期化
	// compressedSensorData compCamData;
	compCamData.width = smdCamera.widthInt;
	compCamData.height = smdCamera.heightInt;
	compCamData.index.resize(smdCamera.index.size());
	compCamData.pt.resize(smdCamera.index.size());
	compCamData.size.resize(smdCamera.index.size());
	int k=0;//compCamData number

	// 圧縮処理
	for(int i = 0; i < smdCamera.index.size(); ){
		int index = smdCamera.index[i].data;
		//データ（1つ目を追加）
		compCamData.index[k].data = index;
		compCamData.pt[k].x = smdCamera.pt[i].x;
		compCamData.pt[k].y = smdCamera.pt[i].y;
		compCamData.pt[k].z = smdCamera.pt[i].z;
		compCamData.size[k].data = 1;
		//indexが等しければさらに追加 : データサイズと点の3次元位置を加算
		for(int j=index+1; j<smdCamera.index.size() && index == smdCamera.index[j].data;j++){
			//データ追加
			compCamData.pt[k].x += smdCamera.pt[i].x;
			compCamData.pt[k].y += smdCamera.pt[i].y;
			compCamData.pt[k].z += smdCamera.pt[i].z;
			compCamData.size[k].data += 1;
		}
		//3次元位置(平均)を算出
		compCamData.pt[k].x /= compCamData.size[k].data;
		compCamData.pt[k].y /= compCamData.size[k].data;
		compCamData.pt[k].z /= compCamData.size[k].data;
		//追加したデータ分だけ移動
		i += compCamData.size[k].data;
	}
	//サイズのリサイズ
	compCamData.index.resize(k);
	compCamData.pt.resize(k);
	compCamData.size.resize(k);
}
void classificationClass::creatMapIndex(){
	//マップセルデータ -> 圧縮データ のインデックス
	//size=mapWidth*mapHeight
	mapIndex.resize(smdCamera.widthInt.data*smdCamera.heightInt.data);
	for(int k=0; k<mapIndex.size(); k++){
		mapIndex[k] = -1;
	}
	// compCamData.index[k] について
	// compCamData.index[k] == マップ上の位置
	// k == データ番号
	// mapIndexについて
	// mapIndex[画像位置] == 圧縮データのデータ参照番号
	// map位置から圧縮データ番号への参照　が目的
	for(int k=0; k<compCamData.index.size(); k++){
		mapIndex[ compCamData.index[k].data ] = k;
	}
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
				count += compCamData.size[ mapIndex[pos] ].data;

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
			int evalCount=(int)( -35*y*y+1200 );
			//評価式よりカウントが小さい時: スキップ
			if(count < evalCount){
				continue;
			}
			//コア点をクラスタに追加
			// データ代入
			//mapIndex[k] <<< k=taskIndex[n] 
			cd.data[clusterNum].size.data += 1;
			cd.data[clusterNum].dens.data += compCamData.size[ mapIndex[taskIndex[n]] ].data;
			cd.data[clusterNum].gc.x += compCamData.pt[ mapIndex[taskIndex[n]] ].x*compCamData.size[ mapIndex[taskIndex[n]] ].data ;
			cd.data[clusterNum].gc.y += compCamData.pt[ mapIndex[taskIndex[n]] ].y*compCamData.size[ mapIndex[taskIndex[n]] ].data ;
			cd.data[clusterNum].gc.z += compCamData.pt[ mapIndex[taskIndex[n]] ].z*compCamData.size[ mapIndex[taskIndex[n]] ].data ;
			cd.data[clusterNum].index[cd.data[clusterNum].size.data - 1].data = taskIndex[n];//マップセルに対するインデックス
			cd.data[clusterNum].pt[cd.data[clusterNum].size.data - 1] = compCamData.pt[ mapIndex[taskIndex[n]] ];
			//追加したデータをマップから削除
			mapIndex[taskIndex[n]] = -1;
			//重複探査防止
			// tempIndex（タスク候補点をタスクに追加）
			for(int m=0; m < tempIndex.size(); m++){
				//追加
				taskIndex[taskSize++] = tempIndex[m];
				//重複探査防止
				searchedIndex[tempIndex[m]] = -1;
			}
		}
		//タスクデータ探査終了
		//平均点算出
		cd.data[clusterNum].gc.x /= cd.data[clusterNum].dens.data;
		cd.data[clusterNum].gc.y /= cd.data[clusterNum].dens.data;
		cd.data[clusterNum].gc.z /= cd.data[clusterNum].dens.data;
		//リサイズ
		cd.data[clusterNum].index.resize(cd.data[clusterNum].size.data);
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
    smdLRF.index.clear();
    smdLRF.pt.clear();
	cd.data.clear();
	compCamData.index.clear();
	compCamData.pt.clear();
	compCamData.size.clear();
}