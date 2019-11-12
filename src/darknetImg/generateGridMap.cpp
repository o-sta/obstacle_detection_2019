#include <obstacle_detection_2019/darknetImg.h>

//旧バージョン
// void darknetImg::generateGridmap(){
//     int row = 0, col = 0, i; //深度画像の行と列
//     float xt, yt, zt;
//     int ch = bridgeImage->image.channels();
//     int rows = bridgeImage->image.rows; //深度画像の行幅
//     int cols = bridgeImage->image.cols; //深度画像の列幅
//     std_msgs::Int32 initial_value;
//     geometry_msgs::Point pt;
//     pt.x = 0; pt.y = 0; pt.z = 0;
//     int mapRow, mapCol; // マップの行と列
//     //初期設定
//     //smdml = boost::make_shared<obstacle_detection_2019::SensorMapDataMultiLayer>();
//     smdml.layer.resize(detection_total);//■error
//     // smdml.layer.resize(10);//
//     int count_layer = 0;
//     for(auto& layer : smdml.layer){
//         ROS_INFO_STREAM("count_layer " << count_layer <<" "<< detection_total);
//         layer.header = smdml.header;
//         layer.width.data = mapWidth;
//         layer.height.data = mapHeight;
//         layer.res.data = mapResolution;
//         layer.widthInt.data = mapCols;
//         layer.heightInt.data = mapRows;
//         layer.index.resize(numberOfCells);
//         layer.size.resize(numberOfCells);
//         layer.pt.resize(numberOfCells);
//         initial_value.data = -1;
//         std::fill(layer.index.begin(), layer.index.end(), initial_value); //std::vectorの全要素を-1で埋める
//         initial_value.data = 0;
//         std::fill(layer.size.begin(), layer.size.end(), initial_value);
//         std::fill(layer.pt.begin(), layer.pt.end(), pt);
//     }
//     //処理
//     int count = 0;
//     int *index;
//     for(row = 0; row < rows; row++){
//         auto bi = bridgeImage->image.ptr<float>(row);
//         auto mi = mask.ptr<char>(row);
//         for(col = 0; col < cols; col++){
//             if(mi[col] > 10){
//                 ROS_WARN_STREAM("invalid mask number ... " << (int)mi[col]);
//                 continue;
//             } 
//             if(mi[col] != 0){
//                 zt = bi[col*ch];
//                 xt = -( ((float)col-(float)cols/2)*zt/f);
//                 if(convertToGrid(xt, zt, mapCol, mapRow) == true){
//                     index = &smdml.layer[mi[col]-1].index[mapRow*mapCols+mapCol].data;
//                     if(index < 0){
//                         *index = count++;
//                     }
//                     yt=((float)rows/2-row)*zt/f+camHeight;//高さ算出
//                     smdml.layer[mi[col]-1].size[*index].data++;
//                     smdml.layer[mi[col]-1].pt[*index].x += xt;
//                     smdml.layer[mi[col]-1].pt[*index].y += yt;
//                     smdml.layer[mi[col]-1].pt[*index].z += zt;
//                 }
//             }
//         }
//     }
// }

void darknetImg::generateGridMap(){
    int max_cell_pts = 0; //最大点数
    const int min_cell_pts = 0; //最小点数
    float xt, yt, zt;
    int ch = bridgeImage->image.channels();
    int rows = bridgeImage->image.rows; //深度画像の行幅
    int cols = bridgeImage->image.cols; //深度画像の列幅
    int mapRow, mapCol; // マップの行と列
    std::vector<std::vector<int>> cell_exists(detection_total, std::vector<int>(numberOfCells, -1)); //セルが存在している場合は配列番号を書き込む
    int exist;
    int target_layer;
    //PersonGridMap初期化 
    std::vector<int> pgm_count(detection_total, 0); //index, size, ptの数
    pgm.header.stamp = bridgeImage->header.stamp;
    pgm.height = mapHeight;
    pgm.width = mapWidth;
    pgm.resolution = mapResolution;
    pgm.layer.resize(detection_total);
    for(auto& layer : pgm.layer){
        layer.index.resize(numberOfCells);
        layer.size.resize(numberOfCells);
        layer.pt.resize(numberOfCells);
    }
    //処理
    for(int row = 0; row < rows; row++){
        auto bi = bridgeImage->image.ptr<float>(row);
        auto mi = mask.ptr<char>(row);
        for(int col = 0; col < cols; col++){
            if(mi[col] > 0){
                zt = bi[col*ch];
                xt = -( ((float)col-(float)cols/2)*zt/f);
                if(convertToGrid(xt, zt, mapCol, mapRow) == true){
                    target_layer = mi[col] - 1;
                    exist = cell_exists[target_layer][mapRow*mapCols+mapCol];
                    if(exist == -1){
                        cell_exists[target_layer][mapRow*mapCols+mapCol] = pgm_count[target_layer];
                        pgm.layer[target_layer].index[pgm_count[target_layer]] = mapRow*mapCols+mapCol;
                        pgm.layer[target_layer].size[pgm_count[target_layer]] = 1;
                        pgm.layer[target_layer].pt[pgm_count[target_layer]].x = -(((float)col-(float)cols/2)*zt/f);
                        pgm.layer[target_layer].pt[pgm_count[target_layer]].y = ((float)rows/2-row)*zt/f+camHeight;
                        pgm.layer[target_layer].pt[pgm_count[target_layer]].z = bi[col*ch];
                        pgm_count[target_layer]++;
                    }else{
                        pgm.layer[target_layer].size[exist]++;
                        pgm.layer[target_layer].pt[exist].x += -(((float)col-(float)cols/2)*zt/f);
                        pgm.layer[target_layer].pt[exist].y += ((float)rows/2-row)*zt/f+camHeight;
                        pgm.layer[target_layer].pt[exist].z += bi[col*ch];
                    }
                }
            }
        }
    }
    for(int i = 0; i < detection_total; i++){
        pgm.layer[i].index.resize(pgm_count[i]);
        pgm.layer[i].size.resize(pgm_count[i]);
        pgm.layer[i].pt.resize(pgm_count[i]);
    }
}

//採用しない方針
// void darknetImg::dimensionalityReductionGridmap(){
//     std_msgs::Int32 initial_value;
//     geometry_msgs::Point pt;
//     pt.x = 0; pt.y = 0; pt.z = 0;
//     int *index, index_smdml;
//     int layer_i = 0; //layerのイテレータ
//     int rowOfCell;
//     int count = 0;
//     smdmlLowDimension.header = smdml.header;
//     smdmlLowDimension.layer.resize(smdml.layer.size());
//     for(auto& layer : smdmlLowDimension.layer){ //全てのSensorMapDataで走査
//         count = 0;
//         layer.header = smdml.header;
//         layer.width.data = mapWidth;
//         layer.height.data = mapHeight;
//         layer.res.data = mapResolution;
//         layer.widthInt.data = 1;
//         layer.heightInt.data = mapRows;
//         layer.index.resize(mapRows);
//         layer.size.resize(mapRows);
//         layer.pt.resize(mapRows);
//         initial_value.data = -1;
//         std::fill(layer.index.begin(), layer.index.end(), initial_value); //std::vectorの全要素を-1で埋める
//         initial_value.data = 0;
//         std::fill(layer.size.begin(), layer.size.end(), initial_value);
//         std::fill(layer.pt.begin(), layer.pt.end(), pt);
//         int mapIndexSize = smdml.layer[layer_i].index.size();
//         for(int cell=0; cell<mapIndexSize; cell++){ //
//             if(smdml.layer[layer_i].index[cell].data < 0){ //smdmlのセルが空の場合はスキップ
//                 continue;
//             }
//             rowOfCell = cell / smdml.layer[layer_i].widthInt.data;
//             index = &layer.index[rowOfCell].data;
//             if(*index < 0){
//                 *index = count++;
//             }
//             index_smdml = smdml.layer[layer_i].index[cell].data;
//             layer.pt[*index].x += smdml.layer[layer_i].pt[index_smdml].x * (double)smdml.layer[layer_i].size[index_smdml].data;
//             layer.pt[*index].y += smdml.layer[layer_i].pt[index_smdml].y * (double)smdml.layer[layer_i].size[index_smdml].data;
//             layer.pt[*index].z += smdml.layer[layer_i].pt[index_smdml].z * (double)smdml.layer[layer_i].size[index_smdml].data;
//             layer.size[*index].data += smdml.layer[layer_i].size[index_smdml].data;
//         }
//         layer.index.resize(count);
//         for(auto index_layer : layer.index){
//             layer.pt[index_layer.data].x /= (double)layer.size[index_layer.data].data;
//             layer.pt[index_layer.data].y /= (double)layer.size[index_layer.data].data;
//             layer.pt[index_layer.data].z /= (double)layer.size[index_layer.data].data;
//         }
//         layer_i++;
//     }
// }


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
