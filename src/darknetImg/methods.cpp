#include <obstacle_detection_2019/darknetImg.h>
// #include <boost/range/algorithm.hpp>

// sensor_callback用コンストラクタ
// darknetImg::darknetImg(/* args */)
// : bb_sub(nhSub, "/darknet_ros/bounding_boxes", 1), image_sub(nhSub, "/robot2/zed_node/left/image_rect_color", 1), sync(MySyncPolicy(10),bb_sub, image_sub)
// {
//     pub = nhPub.advertise<sensor_msgs::Image>("/dphog/boximage", 1);
//     sync.registerCallback(boost::bind(&darknetImg::sensor_callback, this, _1, _2));
// }

// sensor_callback2用コンストラクタ
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

    //枠線描画用コード　debugの方に移動予定
    // bounding_boxedに書かれた枠の描画
    auto iter = bb->bounding_boxes.begin();
    for (; iter != bb->bounding_boxes.end(); ++iter){
        cv::rectangle(bridgeImage->image, cv::Point(iter->xmin, iter->ymin), cv::Point(iter->xmax, iter->ymax), cv::Scalar(0, 0, 200), 5, 8);
    }
    // パブリッシュ(debgu移行予定)
    pub.publish(bridgeImage->toImageMsg());

    boundingBoxesMsg = *bb;
    // if (imageRows != bridgeImage->image.rows || imageCols != bridgeImage->image.cols){
    //     imageRows = bridgeImage->image.rows;
    //     imageCols = bridgeImage->image.cols;
    //     setMaskSize();
    // }
    // if (bb->bounding_boxes.size() > 0){
    //     ROS_INFO_STREAM("pickUpGroundPointCandidates");
    //     pickUpGroundPointCandidates();
    //     ROS_INFO_STREAM("estimateGroundCoefficients");
    //     estimateGroundCoefficients();
    //     ROS_INFO_STREAM("removeGroundPoints");
    //     removeGroundPoints();
    //     ROS_INFO_STREAM("trimPoints");
    //     trimPoints();
    //     ROS_INFO_STREAM("generateGridmap");
    //     generateGridmap();
    //     ROS_INFO_STREAM("dimensionalityReductionGridmap");
    //     dimensionalityReductionGridmap();
    //     ROS_INFO_STREAM("classifyPoints");
    //     classifyPoints();
    //     ROS_INFO_STREAM("estimatePersonPosition");
    //     estimatePersonPosition();
    //     ROS_INFO_STREAM("predictPersonPosition");
    //     predictPersonPosition();
    //     ROS_INFO_STREAM("iteration finished");
    // }else{
    //     ROS_INFO_STREAM("not person detection");
    // }
    // clearMsg(smdml);
    ROS_INFO_STREAM("----------------------------------------");
}



void darknetImg::generateGridmap(){
    int row = 0, col = 0, i; //深度画像の行と列
    float xt, yt, zt;
    int ch = bridgeImage->image.channels();
    int rows = bridgeImage->image.rows; //深度画像の行幅
    int cols = bridgeImage->image.cols; //深度画像の列幅
    std_msgs::Int32 initial_value;
    geometry_msgs::Point pt;
    pt.x = 0; pt.y = 0; pt.z = 0;
    int mapRow, mapCol; // マップの行と列
    //初期設定
    //smdml = boost::make_shared<obstacle_detection_2019::SensorMapDataMultiLayer>();
    // smdml.layer.resize(boundingBoxesMsg.bounding_boxes.size());//■error
    smdml.layer.resize(10);//
    for(auto& layer : smdml.layer){
        layer.header = smdml.header;
        layer.width.data = mapWidth;
        layer.height.data = mapHeight;
        layer.res.data = mapResolution;
        layer.widthInt.data = mapCols;
        layer.heightInt.data = mapRows;
        layer.index.resize(numberOfCells);
        layer.size.resize(numberOfCells);
        layer.pt.resize(numberOfCells);
        initial_value.data = -1;
        std::fill(layer.index.begin(), layer.index.end(), initial_value); //std::vectorの全要素を-1で埋める
        initial_value.data = 0;
        std::fill(layer.size.begin(), layer.size.end(), initial_value);
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
                    index = &smdml.layer[mi[col]-1].index[mapRow*mapCols+mapCol].data;
                    if(index < 0){
                        *index = count++;
                    }
                    yt = ((float)rows/2-row)*zt/f;
                    smdml.layer[mi[col]-1].size[*index].data++;
                    smdml.layer[mi[col]-1].pt[*index].x += xt;
                    smdml.layer[mi[col]-1].pt[*index].y += yt;
                    smdml.layer[mi[col]-1].pt[*index].z += zt;
                }
            }
        }
    }
}


void darknetImg::dimensionalityReductionGridmap(){
    std_msgs::Int32 initial_value;
    geometry_msgs::Point pt;
    pt.x = 0; pt.y = 0; pt.z = 0;
    int *index, index_smdml;
    int layer_i = 0; //layerのイテレータ
    int rowOfCell;
    int count = 0;
    smdmlLowDimension.header = smdml.header;
    smdmlLowDimension.layer.resize(smdml.layer.size());
    for(auto& layer : smdmlLowDimension.layer){ //全てのSensorMapDataで走査
        count = 0;
        layer.header = smdml.header;
        layer.width.data = mapWidth;
        layer.height.data = mapHeight;
        layer.res.data = mapResolution;
        layer.widthInt.data = 1;
        layer.heightInt.data = mapRows;
        layer.index.resize(mapRows);
        layer.size.resize(mapRows);
        layer.pt.resize(mapRows);
        initial_value.data = -1;
        std::fill(layer.index.begin(), layer.index.end(), initial_value); //std::vectorの全要素を-1で埋める
        initial_value.data = 0;
        std::fill(layer.size.begin(), layer.size.end(), initial_value);
        std::fill(layer.pt.begin(), layer.pt.end(), pt);
        int mapIndexSize = smdml.layer[layer_i].index.size();
        for(int cell=0; cell<mapIndexSize; cell++){ //
            if(smdml.layer[layer_i].index[cell].data < 0){ //smdmlのセルが空の場合はスキップ
                continue;
            }
            rowOfCell = cell / smdml.layer[layer_i].widthInt.data;
            index = &layer.index[rowOfCell].data;
            if(*index < 0){
                *index = count++;
            }
            index_smdml = smdml.layer[layer_i].index[cell].data;
            layer.pt[*index].x += smdml.layer[layer_i].pt[index_smdml].x * (double)smdml.layer[layer_i].size[index_smdml].data;
            layer.pt[*index].y += smdml.layer[layer_i].pt[index_smdml].y * (double)smdml.layer[layer_i].size[index_smdml].data;
            layer.pt[*index].z += smdml.layer[layer_i].pt[index_smdml].z * (double)smdml.layer[layer_i].size[index_smdml].data;
            layer.size[*index].data += smdml.layer[layer_i].size[index_smdml].data;
        }
        layer.index.resize(count);
        for(auto index_layer : layer.index){
            layer.pt[index_layer.data].x /= (double)layer.size[index_layer.data].data;
            layer.pt[index_layer.data].y /= (double)layer.size[index_layer.data].data;
            layer.pt[index_layer.data].z /= (double)layer.size[index_layer.data].data;
        }
        layer_i++;
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
    //設定データ
    auto layer = smdmlLowDimension.layer[0];
    cd.header = layer.header;
    cd.width = layer.width;
    cd.height = layer.height;
    cd.res = layer.res;
    cd.widthInt = layer.widthInt;
    cd.heightInt = layer.heightInt;
    cd.cp = layer.cp;
    cd.size.data = 0;
    cd.data.resize(layer.index.size()); //障害物数だけ仮確保
    // mapIndex = -1 -> searched or not include obstacle point
    for(auto layer : smdmlLowDimension.layer){ //探査済み領域で探査開始
        std::vector<int> mapIndex; //探査済み領域の場合は-1が代入される
        mapIndex.resize(layer.index.size());
        for(int k=0; k < mapIndex.size(); k++){
            mapIndex[k] = layer.index[k].data;
        }
        // DBSCAN開始
        ROS_INFO("DBSCAN start");
        for(int k = 0; k < mapIndex.size(); k++){ //各マップセルで捜査(クラスタ候補のコア点探し)
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
            for(int n=0; n < taskSize; n++){ //コア点候補を走査
                int count_points = 0;//密度（点の数）
                int robotX = (taskIndex[n] % layer.widthInt.data) - layer.widthInt.data/2;
                int robotY = -( (taskIndex[n] / layer.widthInt.data) - layer.heightInt.data/2 );
                float tempX = robotX * layer.res.data;
                float tempY = robotY * layer.res.data;

                std::vector<int> tempIndex;//Window探索中のみ使用するIndex
                tempIndex.resize(cellsInWindow.size());
                int tempSize = 0;

                //窓内に含まれているセルを列挙して、点の数を数える
                //また、コア点として探査されていない点を候補に追加
                for(int i=0; i < cellsInWindow.size(); i++){                    //窓内セルに対して走査
                    int pos = taskIndex[n] + cellsInWindow[i];                  //窓内セルの座標とコア点座標を重畳
                    if (pos < 0 || pos >= layer.heightInt.data){ continue; }    //探査対象がマップ範囲外のときはスキップ
                    if(mapIndex[pos] < 0){ continue; }                          //インデックスが存在しない場合はスキップ
                    count_points += layer.size[ mapIndex[pos] ].data;           //クラスタかどうか判定するため、点の数をカウント
                    if(searchedIndex[pos] < 0){ continue; }                     //探査済みの場合はスキップ
                    tempIndex[tempSize++] = pos;                                //次の探査点（コア点）候補に追加（クラスタ判定されたらtaskIndexに追加する）
                }
                tempIndex.resize(tempSize);
                if(count_points < minPts){ continue; } //クラスタ判定、窓内の店の数が基準を超えていない場合以下スキップ
                //コア点をクラスタに追加
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
                    //重複探査防止（コア点候補へ2回以上追加されるのを防止）
                    searchedIndex[tempIndex[m]] = -1;
                }
            } //1つのクラスタ完成

            if(cd.data[clusterNum].size.data==0){ //クラスタが構成されていない場合
                mapIndex[cd.data[clusterNum].index[0].data] = -1; //■これ不明
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
    }
    // DBSCAN終了
    //データリサイズ
    cd.data.resize(cd.size.data);
}


void darknetImg::estimatePersonPosition(){
    int i;
    for(auto& ce : cd.data){
        ROS_INFO_STREAM("person[" << i++ <<"] : [" << ce.gc.x << ce.gc.y << ce.gc.z << "]");
    }
}


void darknetImg::predictPersonPosition(){
    
}

void darknetImg::clearMsg(obstacle_detection_2019::SensorMapDataMultiLayer& smdml_msg){
    for(auto smd_msg : smdml_msg.layer){
    smd_msg.header.frame_id.clear();
    smd_msg.index.clear();
    smd_msg.pt.clear();
    smd_msg.size.clear();
    }
}
