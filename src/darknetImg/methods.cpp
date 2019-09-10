#include <obstacle_detection_2019/darknetImg.h>

// sensor_callback用コンストラクタ
// darknetImg::darknetImg(/* args */)
// : bb_sub(nhSub, "/darknet_ros/bounding_boxes", 1), image_sub(nhSub, "/robot2/zed_node/left/image_rect_color", 1), sync(MySyncPolicy(10),bb_sub, image_sub)
// {
//     pub = nhPub.advertise<sensor_msgs::Image>("/dphog/boximage", 1);
//     sync.registerCallback(boost::bind(&darknetImg::sensor_callback, this, _1, _2));
// }

// sensor_callback2用コンストラクタ
darknetImg::darknetImg(/* args */)
: 
bb_sub(nhSub, "/darknet_ros/bounding_boxes", 1), 
image_sub(nhSub, "/robot2/zed_node/left/image_rect_color", 1), 
sync(MySyncPolicy(10),bb_sub, image_sub),
is_size_initialized(false),
mask(1,1,CV_8UC1),
ground_points(new pcl::PointCloud<pcl::PointXYZ>)
{
    pub = nhPub.advertise<sensor_msgs::Image>("/dphog/boximage", 1);
    sync.registerCallback(boost::bind(&darknetImg::sensor_callback, this, _1, _2));
    mapWidth = 8;
    mapHeight = 8;
    mapResolution = 0.05;
    mapRows = (int)(mapHeight/mapResolution);
    mapCols = (int)(mapWidth/mapResolution);
    numberOfCells = mapRows*mapCols;
}

darknetImg::~darknetImg()
{
    
}

void darknetImg::sensor_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bb, const sensor_msgs::Image::ConstPtr& image)
{
    // debug移行予定
    ROS_INFO_STREAM("callback : " << "[bb : " << bb->header.stamp.toSec() << " ]," 
                                  << "[img : " << image->header.stamp.toSec() << " ],"
                                  << "[diff : " << (bb->header.stamp - image->header.stamp).toSec() << "]");
    // image から cvbridgeImage に変換する
    try{
        bridgeImage=cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);
    }
    catch(cv_bridge::Exception& e) {//エラー処理
        ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.",image->encoding.c_str());
        return;
    }
    // bounding_boxedに書かれた枠の描画
    auto iter = bb->bounding_boxes.begin();
    for (; iter != bb->bounding_boxes.end(); ++iter){
        cv::rectangle(bridgeImage->image, cv::Point(iter->xmin, iter->ymin), cv::Point(iter->xmax, iter->ymax), cv::Scalar(0, 0, 200), 5, 8);
    }
    // パブリッシュ(debgu移行予定)
    pub.publish(bridgeImage->toImageMsg());
}

void darknetImg::pickUpGroundPointCandidates(){
    float xt, yt, zt; //点の座標（テンプレート）
    pcl::PointXYZ pt; //点の座標（ポイントクラウド型テンプレート）
    int rows = bridgeImage->image.rows; //深度画像の行
    int cols = bridgeImage->image.cols; //深度画像の列
    int candidateNum = 0;//床面候補点の数(ground_pointsのサイズ)
    ground_points->points.resize(rows/2*cols);//候補点の最大値でリサイズ
    int ch = bridgeImage->image.channels(); //チャンネル数
    //床面候補点抽出処理
    for(int i = rows/2+1; i<rows; i++){//画像下半分を走査
        float *p = bridgeImage->image.ptr<float>(i); //i行1列目のアドレス
        for(int j = 0; j < cols; j++){//走査}
            zt = p[j*ch];
            if(zt > 0.5 && !std::isinf(zt)){
                yt = ((float)rows/2-i)*zt/f; //高さ
                if(std::abs(yt+camHeight) < groundCandidateY){ //高さがgroundCandidateY未満の時
                    xt = -( ((float)i-(float)cols/2)*zt/f-camHeight );
                    pt.x=zt;
                    pt.y=xt;
                    pt.z=yt;
                    ground_points->points[candidateNum++] = pt;
                }
            }
        }
    }
    ground_points->points.resize(candidateNum);
    ground_points->width=ground_points->points.size();
    ground_points->height=1;
    ROS_INFO_STREAM("ground_points->points.size():"<<ground_points->points.size()<<"\n");
}

void darknetImg::estimateGroundCoefficients(){
    seg.setInputCloud(ground_points);
	seg.segment(*inliers, *coefficients);
    a=coefficients->values[0];
    b=coefficients->values[1];
    c=coefficients->values[2];
    d=coefficients->values[3];
    //床面式の確認と再設定
	//const float ground_th=0.20;//床面式から高さground_thまでのデータを床面とする
    if(std::abs(d-camHeight>=0.15)){//推定値があまりにも変な場合
        //事前に決めた値を使用
		a=-0.08;
		b=0;
		c=1;
		d=camHeight-ground_th;
	}
	else{
		d-=ground_th;
	}
    ROS_INFO_STREAM("Model coefficients: " << a << " "
                                    << b << " "
                                    << c << " "
                                    << d << "\n");
}

void darknetImg::removeGroundPoints(){
    int row = 0, col = 0;
    float xt, yt, zt;
    int ch = bridgeImage->image.channels();
    int rows = bridgeImage->image.rows; //深度画像の行
    int cols = bridgeImage->image.cols; //深度画像の列
    //マスクのりサイズ
    if(is_size_initialized == false){
        cv::resize(mask, mask, cv::Size(rows,cols));
    }
    for(row = 0; row < rows; row++){
        float *bi = bridgeImage->image.ptr<float>(row);
        char *mi = mask.ptr<char>(row);
        for(col = 0; col < cols; col++){
            zt = bi[col*ch];
            if(zt>0.5&&!std::isinf(zt)){
                yt=((float)cols/2-row)*zt/f;//高さ算出
                //xt=-( ((float)row-(float)rows/2)*zt/f-camHeight );
                float y_ground=(-a*zt-b*xt-d)/c;//床面の高さを算出
                //高さが床面以上であればmask値を1に
                yt > y_ground ? mi[col] = 1 : mi[col] = 0;
            } else{
                mi[col] = 0;
            }
        }
    }
}

void darknetImg::trimPoints(){
    int i = 0;
    for(const auto& bb : boundingBoxesMsg.bounding_boxes){
        cv::rectangle(mask, cv::Point(bb.xmin, bb.ymin), cv::Point(bb.xmax, bb.ymax), cv::Scalar(++i), -1, CV_FILLED);
    }
}

void darknetImg::generateGridmap(){
    int row = 0, col = 0, i; //深度画像の行と列
    float xt, yt, zt;
    int ch = bridgeImage->image.channels();
    int rows = bridgeImage->image.rows; //深度画像の行幅
    int cols = bridgeImage->image.cols; //深度画像の列幅
    geometry_msgs::Point pt;
    pt.x = 0; pt.y = 0; pt.z = 0;
    int mapRow, mapCol; // マップの行と列
    //初期設定
    smdml.layer.resize(boundingBoxesMsg.bounding_boxes.size());
    for(auto& layer : smdml.layer){
        layer.index.resize(rows*cols);
        layer.header = smdml.header;
        layer.width.data = mapWidth;
        layer.height.data = mapHeight;
        layer.res.data = mapResolution;
        layer.widthInt.data = mapCols;
        layer.heightInt.data = mapRows;
        layer.index.resize(numberOfCells);
        layer.size.resize(numberOfCells);
        layer.pt.resize(numberOfCells);
        std::fill(layer.index.begin(), layer.index.end(), -1); //std::vectorの全要素を-1で埋める
        std::fill(layer.size.begin(), layer.size.end(), 0);
        std::fill(layer.pt.begin(), layer.pt.end(), pt);
    }
    //処理
    int count = 0;
    int *index;
    for(row = 0; row < rows; row++){
        float *bi = bridgeImage->image.ptr<float>(row);
        char *mi = mask.ptr<char>(row);
        for(col = 0; col < cols; col++){
            if(mi[col] != 0){
                zt = bi[col*ch];
                xt=-(((float)row-(float)rows/2)*zt/f-camHeight);
                if(convertToGrid(xt, zt, mapCol, mapRow) == true){
                    index = &smdml.layer[mi[col]].index[mapRow*mapCols+mapCol].data;
                    if(index < 0){
                        *index = count++;
                    }
                    yt = ((float)rows/2-row)*zt/f;
                    smdml.layer[mi[col]].size[*index].data++;
                    smdml.layer[mi[col]].pt[*index].x += xt;
                    smdml.layer[mi[col]].pt[*index].y += yt;
                    smdml.layer[mi[col]].pt[*index].z += zt;
                }
            }
        }
    }
}

bool darknetImg::convertToGrid(const float& x,const float& y,int& xg,int& yg){
	//マップ上の中心座標(ふつうは　センサ位置＝マップ中心座標　のため　cx=cy=0)
	float cx=0;
	float cy=0;
    //マップ原点座標を画像原点座標(左上)に移動
	float map_x = mapWidth/2 + (x - cx);
	float map_y = mapHeight/2 + ( -(y - cy) );
    //マップ外のデータの際はreturn false
	if(map_x<0 || map_x>mapWidth)
		return false;
	if(map_y<0 || map_y>mapHeight)
		return false;
    //ピクセルデータに変換
	xg = (int)(map_x/mapResolution);
	yg = (int)(map_y/mapResolution);
    //変換成功
	return true;
}

void darknetImg::classifyPoints(){
    for(auto layer : smdml.layer){ //各レイヤーで捜査
        //設定データ
        cd.header = layer.header;
        cd.width = layer.width;
        cd.height = layer.height;
        cd.res = layer.res;
        cd.widthInt = layer.widthInt;
        cd.heightInt = layer.heightInt;
        cd.cp = layer.cp;
        cd.size.data = 0;
        //障害物データ数確保
        cd.data.resize(layer.index.size());
        // mapIndex = -1 -> searched or not include obstacle point
        std::vector<int> mapIndex;
        mapIndex.resize(layer.index.size());
        for(int k=0; k < mapIndex.size(); k++){
            mapIndex[k] = layer.index[k].data;
        }
        // DBSCAN開始
        ROS_INFO("DBSCAN start");
        for(int k = 0; k < mapIndex.size(); k++){ //各マップセルで捜査
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
            // DBSCAN開始
            for(int n=0; n < taskSize; n++){
                int count = 0;//密度（点の数）
                int robotX = (taskIndex[n] % layer.widthInt.data) - layer.widthInt.data/2;
                int robotY = -( (taskIndex[n] / layer.widthInt.data) - layer.heightInt.data/2 );
                float tempX = robotX * layer.res.data;
                float tempY = robotY * layer.res.data;

                // int angle;
                // if(tempX==0){
                //     angle = 0;
                // }
                // else
                // {
                //     angle = -((int)( atan2(tempY,tempX)/M_PI *180) )+90 ;
                // }
                // if(angle < minCamDeg || angle > maxCamDeg){
                //     ROS_INFO("angle Error: %d",angle);
                //     continue;
                // }
                // //使用窓番号選択
                // int winNum = selectWindow(angle);
                // // ROS_INFO("winNum:%d",winNum);
                // if( (int)winIndex2.size() <= winNum){
                //     ROS_INFO("over winIndex2.size()");
                //     continue;
                // }
                // ROS_INFO("angle: %d, winIndex2[%d].size():%d",angle, winNum,(int)winIndex2[winNum].size());
                std::vector<int> cells_in_window; //窓内のセル
                cells_in_window.resize(winindex costs)
                std::vector<int> tempIndex;//Window探索中のみ使用するIndex
                tempIndex.resize(winIndex2[winNum].size());
                int tempSize = 0;


                //窓内に含まれているセルを列挙(独自)
                for(int )


                //窓内に含まれているセルを列挙
                // std::cout<<"task("<<taskIndex[n] % smdCamera.widthInt.data<<","<<taskIndex[n] / smdCamera.widthInt.data<<")-->searching-->\n";
                for(int m=0; m < winIndex2[winNum].size(); m++){
                    int pos = taskIndex[n] + winIndex2[winNum][m];
                    int posW = taskIndex[n] % layer.widthInt.data;
                    int moveW = winIndex2[winNum][m] % layer.widthInt.data;
                    //デバッグ用
                    // ROS_INFO("%d.size(%d) pos, posW, moveW: (%d, %d, %d)",
                    // winNum, (int)(winIndex2[winNum].size()),pos, posW, moveW);
                    int taskX= taskIndex[n] % layer.widthInt.data;
                    int taskY= taskIndex[n] / layer.widthInt.data;
                    int moveX= winIndex2[winNum][m] % layer.widthInt.data;
                    int moveY= winIndex2[winNum][m] / layer.widthInt.data;
                    if(layer.widthInt.data - moveX < moveX){
                        moveX = moveX - layer.widthInt.data;
                        moveY = moveY + 1;
                    }
                    else if(layer.widthInt.data + moveX < -moveX){
                        moveX = moveX + layer.widthInt.data;
                        moveY = moveY - 1;
                    }
                    // ROS_INFO("task(%d,%d), move(%d, %d)", taskX, taskY, moveX, moveY);
                    int difX = taskX + moveX;
                    int difY = taskY + moveY;
                    //ここの処理, 後で確認
                    int difW = posW + moveW;
                    // if(pos < 0 || pos >= (int)mapIndex.size()
                    // 	|| difW < 0 || difW > layer.widthInt.data ){
                    if(difX < 0 || difX >= layer.widthInt.data
                        || difY < 0 || difY >= layer.heightInt.data ){
                            //マップ範囲外検索
                            // ROS_INFO("outrange map");
                            continue;
                    }
                    //セルにデータがない場合：スキップ
                    if(mapIndex[pos] < 0){
                        // ROS_INFO("no data cell");
                        continue;
                    }
                    // std::cout<<"get-->pos("<<pos % layer.widthInt.data<<","<<pos / layer.widthInt.data<<")\n";
                    //点の数（密度）を追加
                    // std::cout<<"-->Index[pos]:"<<mapIndex[pos]<<std::endl;
                    // std::cout<<"-->dens:"<<layer.size[ mapIndex[pos] ]<<std::endl;
                    // std::cout<<"-->pos:"<<layer.pt[ mapIndex[pos] ]<<std::endl;
                    count += layer.size[ mapIndex[pos] ].data;

                    //探査済みの場合：スキップ
                    if(searchedIndex[pos]< 0){
                        // ROS_INFO("already searched");
                        continue;
                    }
                    //次のタスク候補点（コア点候補）として追加
                    tempIndex[tempSize++] = pos;
                }

                // std::cout<<"--> end search \n";
                //リサイズ
                tempIndex.resize(tempSize);

                //点の数（密度）を評価
                // (smdCamera.height/2+smdCamera.cp) : センサからマップの上端までの距離
                // taskIndex[n] / smdCamera.widthInt * smdCamera.res : マップ上端から障害物セルまでの距離
                float y=(layer.height.data/2.0+layer.cp.y) - taskIndex[n] / layer.widthInt.data * layer.res.data;
                //奥行yに対する評価式らしい
                // int minPts=(int)( -35*y*y+1200 );
                // int minPts=10;
                //評価式よりカウントが小さい時: スキップ
                // ROS_INFO("%d < %d",count,minPts);
                if(count < minPts){
                    continue;
                }
                //コア点をクラスタに追加
                // データ代入
                //mapIndex[k] <<< k=taskIndex[n] 
                cd.data[clusterNum].size.data += 1;
                cd.data[clusterNum].dens.data += layer.size[ mapIndex[taskIndex[n]] ].data;
                cd.data[clusterNum].gc.x += layer.pt[ mapIndex[taskIndex[n]] ].x*layer.size[ mapIndex[taskIndex[n]] ].data ;
                cd.data[clusterNum].gc.y += layer.pt[ mapIndex[taskIndex[n]] ].y*layer.size[ mapIndex[taskIndex[n]] ].data ;
                cd.data[clusterNum].gc.z += layer.pt[ mapIndex[taskIndex[n]] ].z*layer.size[ mapIndex[taskIndex[n]] ].data ;
                cd.data[clusterNum].index[cd.data[clusterNum].size.data - 1].data = taskIndex[n];//マップセルに対するインデックス, 1データ番号->マップセル
                cd.data[clusterNum].pt[cd.data[clusterNum].size.data - 1] = layer.pt[ mapIndex[taskIndex[n]] ];
                //重複探査防止
                // tempIndex（タスク候補点をタスクに追加）
                // ROS_INFO("tempIndex.size(): %d",(int)tempIndex.size());
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
    
}

void classificationClass::newClassificationDBSCAN(){//カメラ

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
	ROS_INFO("DBSCAN start");
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

		// ROS_INFO("for(int n=0; n < taskSize; n++");
		for(int n=0; n < taskSize; n++){
			int count = 0;//密度（点の数）
			int robotX = (taskIndex[n] % smdCamera.widthInt.data) - smdCamera.widthInt.data/2;
			int robotY = -( (taskIndex[n] / smdCamera.widthInt.data) - smdCamera.heightInt.data/2 );
			float tempX = robotX * smdCamera.res.data;
			float tempY = robotY * smdCamera.res.data;
			// ROS_INFO("x,y:%f,%f",tempX,tempY);
			int angle;
			if(tempX==0){
				angle = 0;
			}
			else
			{
				angle = -((int)( atan2(tempY,tempX)/M_PI *180) )+90 ;
			}
			if(angle<minCamDeg || angle > maxCamDeg){
				ROS_INFO("angle Error: %d",angle);
				continue;
			}
			//使用窓番号選択
			int winNum = selectWindow(angle);
			// ROS_INFO("winNum:%d",winNum);
			if( (int)winIndex2.size() <= winNum){
				ROS_INFO("over winIndex2.size()");
				continue ;
			}
			// ROS_INFO("angle: %d, winIndex2[%d].size():%d",angle, winNum,(int)winIndex2[winNum].size());
			std::vector<int> tempIndex;//Window探索中のみ使用するIndex
			tempIndex.resize(winIndex2[winNum].size());
			int tempSize = 0;
			//窓内を探索
			// std::cout<<"task("<<taskIndex[n] % smdCamera.widthInt.data<<","<<taskIndex[n] / smdCamera.widthInt.data<<")-->searching-->\n";
			for(int m=0; m < winIndex2[winNum].size(); m++){
				int pos = taskIndex[n] + winIndex2[winNum][m];
				int posW = taskIndex[n] % smdCamera.widthInt.data;
				int moveW = winIndex2[winNum][m] % smdCamera.widthInt.data;
				//デバッグ用
				// ROS_INFO("%d.size(%d) pos, posW, moveW: (%d, %d, %d)",
				// winNum, (int)(winIndex2[winNum].size()),pos, posW, moveW);
				int taskX= taskIndex[n] % smdCamera.widthInt.data;
				int taskY= taskIndex[n] / smdCamera.widthInt.data;
				int moveX= winIndex2[winNum][m] % smdCamera.widthInt.data;
				int moveY= winIndex2[winNum][m] / smdCamera.widthInt.data;
				if(smdCamera.widthInt.data - moveX < moveX){
					moveX = moveX - smdCamera.widthInt.data;
					moveY = moveY + 1;
				}
				else if(smdCamera.widthInt.data + moveX < -moveX){
					moveX = moveX + smdCamera.widthInt.data;
					moveY = moveY - 1;
				}
				// ROS_INFO("task(%d,%d), move(%d, %d)", taskX, taskY, moveX, moveY);
				int difX = taskX + moveX;
				int difY = taskY + moveY;
				//ここの処理, 後で確認
				int difW = posW + moveW;
				// if(pos < 0 || pos >= (int)mapIndex.size()
				// 	|| difW < 0 || difW > smdCamera.widthInt.data ){
				if(difX < 0 || difX >= smdCamera.widthInt.data
					|| difY < 0 || difY >= smdCamera.heightInt.data ){
						//マップ範囲外検索
						// ROS_INFO("outrange map");
						continue;
				}
				//セルにデータがない場合：スキップ
				if(mapIndex[pos] < 0){
					// ROS_INFO("no data cell");
					continue;
				}
				// std::cout<<"get-->pos("<<pos % smdCamera.widthInt.data<<","<<pos / smdCamera.widthInt.data<<")\n";
				//点の数（密度）を追加
				// std::cout<<"-->Index[pos]:"<<mapIndex[pos]<<std::endl;
				// std::cout<<"-->dens:"<<smdCamera.size[ mapIndex[pos] ]<<std::endl;
				// std::cout<<"-->pos:"<<smdCamera.pt[ mapIndex[pos] ]<<std::endl;
				count += smdCamera.size[ mapIndex[pos] ].data;

				//探査済みの場合：スキップ
				if(searchedIndex[pos]< 0){
					// ROS_INFO("already searched");
					continue;
				}
				//次のタスク候補点（コア点候補）として追加
				tempIndex[tempSize++] = pos;
			}
			// std::cout<<"--> end search \n";
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
			// ROS_INFO("%d < %d",count,minPts);
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
			// ROS_INFO("tempIndex.size(): %d",(int)tempIndex.size());
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


void darknetImg::estimatePersonPosition(){

}

void darknetImg::predictPersonPosition(){
    
}

