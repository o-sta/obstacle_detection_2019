#include<obstacle_detection_2019/measurementVelocity.h>

//subscribe
void measurementVelocity::subscribeClusterData(){//クラスタデータ受信
	queue1.callOne(ros::WallDuration(1));
}
void measurementVelocity::cluster_callback(const obstacle_detection_2019::ClassificationData::ConstPtr& msg)
{
    //データをコピー
    curClstr.header = msg->header;
    curClstr.width = msg->width;
    curClstr.height = msg->height;
    curClstr.res = msg->res;
    curClstr.widthInt = msg->widthInt;
    curClstr.heightInt = msg->heightInt;
    curClstr.cp = msg->cp;
    curClstr.size = msg->size;
    curClstr.data = msg->data;
}
void measurementVelocity::subscribeImageMatchingData(){//画像マッチングデータ受信
	queue2.callOne(ros::WallDuration(1));
}
void measurementVelocity::matching_callback(const obstacle_detection_2019::ImageMatchingData::ConstPtr& msg)
{
    //データをコピー
    matchData.header = msg->header;
    matchData.width = msg->width;
    matchData.height = msg->height;
    matchData.res = msg->res;
    matchData.widthInt = msg->widthInt;
    matchData.heightInt = msg->heightInt;
    matchData.cp = msg->cp;
    matchData.index = msg->index;
    matchData.data = msg->data;
}
bool measurementVelocity::isPrvClstr(){
	if(prvClstr.header.seq > 0 ){
		return true;
	}
	return false;
}
void measurementVelocity::creatClstrMap(){
	//マップセルに対応するクラスタ番号が記載
	// std::vector<int> curClstMap;
	curClstrMap.assign(curClstr.widthInt * curClstr.heightInt , -1);

	//マップの各セルにクラスタ番号を記載
	for(int i=0; i< curClstr.size; i++){
		for(int k=0; k<curClstr.data[i].index.size(); k++){
			curClstrMap[ curClstr.data[i].index[k] ] = i;//セルにクラスタ番号を記述
		}
	}
}
void measurementVelocity::renewClstrData(){
	//データクリア（必要かどうかは不明）
	prvClstr.data.clear();
	prvClstrMap.clear();
	//データ更新
	prvClstrMap　= curClstrMap;//クラスタマップ
	prvClstr = curClstr;//クラスタデータ
	//
	//データクリア（必要かどうかは不明）
	curClstr.data.clear();
	curClstrMap.clear();
}
void measurementVelocity::matchingClstr(){//（途中）
	//現在のクラスタ -> 1つ前の処理でのクラスタ に対するマッチングを取る
	//画像マッチングスコア
	//行：curClstrのクラスタ数, 列：preClstrのクラスタ数, 値0で初期化
	std::vector<std::vector<int>> imageMathingScore(curClstr.size, std::vector<int>(preClstr.size, 0));
	//matchData（画像マッチングデータ）を走査
	for(int k = 0; k < matchData.index.size(); k++){//widthInt * heightInt
		int prvPos = matchData.index[k];
		int curPos = prvPos + matchData.data[prvPos].x 
			+ matchData.data[prvPos].y * matchData.widthInt;
		imageMathingScore[ curClstrMap[curPos] ][ prvClstrMap[prvPos] ] ++;//カウントアップ
	}
	//位置マッチングスコア
	//重心位置の差
	std::vector<std::vector<float>> posMathingScore(curClstr.size, std::vector<float>(preClstr.size, 0));
	for(int k=0; k < curClstr.size; k++){
		for(int i=0; i< preClstr.size; i++){
			float disX = curClstr.data[k].gc.x - preClstr.data[k].gc.x;
			float disY = curClstr.data[k].gc.y - preClstr.data[k].gc.y;
			float disZ = curClstr.data[k].gc.z - preClstr.data[k].gc.z;
			float dis = std::sqrt( disX*disX +disY*disY + disZ*disZ );
			posMathingScore[k][i] = dis;
		}
	}
	//サイズマッチングスコア
	//サイズ（クラスタ内の点数）の差
	std::vector<std::vector<int>> sizeMathingScore(curClstr.size, std::vector<int>(preClstr.size, 0));
	for(int k=0; k < curClstr.size; k++){
		for(int i=0; i< preClstr.size; i++){
			int curSize = curClstr.data[k].dens;
			int prvSize = prvClstr.data[k].dens;
			sizeMathingScore[k][i] = std::abs(curSize - prvSize);
		}
	}
	//マッチング評価
	// std::vector<int> matchResult(curClstr.size, -1);
	matchResult.assign(curClstr.size , -1);
	for(int k=0; k < curClstr.size; k++){
		//
		//例外除去
		//マップセルの占有マスが少ない（1マス以下）
		if(curClstr.size <= 1){
			continue;
		}
		double evMax = -1;//最大評価値
		int matchNum = -1;
		for(int i=0; i< preClstr.size; i++){
			//例外除去
			//マップセルの占有マスが少ない（1マス以下）
			if(prvClstr.size <= 1){
				continue;
			}
			//重心位置の差が1m以上
			//--移動距離
			if(posMathingScore[i][k] >= 1.0){
				continue;
			}
			double ev;//評価値
			//評価式 : 要検討
			ev = imageMathingScore[k][i] //画像マッチングスコア
				+ (1 / (1 + posMathingScore[i][k]) ) //重心位置マッチングスコア
				+ (1 / (2 * (1 + sizeMathingScore[i][k])) );//サイズマッチングスコア
			//評価値が最大評価値よりも大きいとき
			if(evMax < ev){
				//ベストマッチング値を更新
				evMax = ev;
				matchNum = i;
			}
		}
		//評価が最もよかったやつをマッチング結果に
		matchResult[k] = matchNum;
	}
}
void measurementVelocity::measurementProcess(){//
	//速度データ付きのクラスタデータ
	// obstacle_detection::classificationVelocityData cvd;
    //データをコピー
    cvd.header = curClstr.header;
    cvd.width = curClstr.width;
    cvd.height = curClstr.height;
    cvd.res = curClstr.res;
    cvd.widthInt = curClstr.widthInt;
    cvd.heightInt = curClstr.heightInt;
    cvd.cp = curClstr.cp;
    cvd.size = curClstr.size;
    cvd.data = curClstr.data;
	//経過時間の計算（速度算出用）
	double dt;
	ros::Duration rosDt = curClstr.header.stamp - prvClstr.header.stamp;
	dt = rosDt.toSec();
	for(int k=0; k < curClstr.size; k++){
		if( matchResult[k] < 0){
			cvd.twist[k].linear.x = 0;
			cvd.twist[k].linear.y = 0;
			cvd.twist[k].linear.z = 0;
			cvd.twist[k].angular.x = 0;
			cvd.twist[k].angular.y = 0;
			cvd.twist[k].angular.z = 0;
		}
		else{
			//速度の計算
			//--移動距離
			float dx = curClstr.data[k].gc.x - prvClstr.data[ matchResult[k] ].gc.x;
			float dy = curClstr.data[k].gc.y - prvClstr.data[ matchResult[k] ].gc.y;
			float dz = curClstr.data[k].gc.z - prvClstr.data[ matchResult[k] ].gc.z;
			//--速度計算
			cvd.twist[k].linear.x = dx/dt;
			cvd.twist[k].linear.y = dy/dt;
			cvd.twist[k].linear.z = dz/dt;
			cvd.twist[k].angular.x = 0;
			cvd.twist[k].angular.y = 0;
			cvd.twist[k].angular.z = 0;
		}
	}
}

void measurementVelocity::publishClassificationData(){//データ送信
    pub.publish(cvd);
}