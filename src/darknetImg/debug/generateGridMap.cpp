#include <obstacle_detection_2019/darknetImgDebug.h>

// void darknetImgDebug::publishGridMap(){
//     pcl::PointCloud<pcl::PointXYZRGB> map;
//     pcl::PointXYZRGB cell;
//     sensor_msgs::PointCloud2 map_msg;
//     int max_cell_pts = 0; //最大点数
//     const int min_cell_pts = 0; //最小点数
//     //初期化
//     map_msg.header.stamp = smdml.header.stamp;
//     map_msg.header.frame_id = "/zed_camera_center";
//     map.points.resize(mapRows * mapCols);
//     map.width=map.points.size();
//     map.height=1;
//     // int max_cell_pts_layer_index; //最大点数を持つセルの所属レイヤー（不要かも）
//     // int max_cell_pts_index; //最大点数を持つセル番号（不要かも）
//     //最大値算出
//     for(auto layer : smdml.layer){
//         for(auto cell_index : layer.index){
//             if(cell_index.data >= 0){
//                 if(layer.size[cell_index.data].data > max_cell_pts){
//                     max_cell_pts = layer.size[cell_index.data].data;
//                     // max_cell_pts_index = cell_index.data;
//                 }
//             }
//         }
//     }
//     int colorIndex;
//     int ptIndex;
//     double gauss;
//     int cell_pts;
//     int index;
//     for(int row = 0; row < mapRows; ++row){
//         for(int col = 0; col < mapCols; ++col){
//             ptIndex = row*mapCols + col;
//             cell_pts = 0;
//             for(auto layer : smdml.layer){
//                 index = layer.index[ptIndex].data;
//                 if(index >= 0){
//                     cell_pts += layer.size[index].data;
//                 }
//             }
//             map.points[ptIndex].x = (double)((int)(mapRows/2) - row)*mapResolution - mapResolution/2;
//             map.points[ptIndex].y = (double)(col-(int)(mapCols/2))*mapResolution + mapResolution/2;
//             map.points[ptIndex].z = 0;
//             colorIndex = serectColor((float)cell_pts, (float)min_cell_pts, (float)max_cell_pts, colorMapGrad.size()/3)*3;
//             map.points[ptIndex].r = (uint8_t)(colorMap[colorIndex] * 255);
//             map.points[ptIndex].g = (uint8_t)(colorMap[colorIndex+1] * 255);
//             map.points[ptIndex].b = (uint8_t)(colorMap[colorIndex+2] * 255);
//         }
//     }

//     pcl::toROSMsg(map, map_msg);
//     map_msg.header.stamp = ros::Time::now();
//     map_msg.header.frame_id = "/map";
//     gridMapPCL_pub.publish(map_msg);
// }

void darknetImgDebug::publishGridMap(){
    pcl::PointCloud<pcl::PointXYZRGB> map;
    //pcl::PointXYZRGB cell;
    sensor_msgs::PointCloud2 map_msg;
    std::vector<int> max_cell_pts(detection_total, 0); //最大点数
    map_msg.header.stamp = pgm.header.stamp;
    map_msg.header.frame_id = "/zed_camera_center";
    map.points.resize(numberOfCells * detection_total);
    map.width=map.points.size();
    map.height=1;
    //最大値算出
    for(int i=0; i < pgm.layer.size(); ++i){
        for(auto cell_pts : pgm.layer[i].size){
            if(max_cell_pts[i] < cell_pts){
                max_cell_pts[i] = cell_pts;
            }
        }
    }
    int colorIndex;
    int ptIndex = 0;
    int cell_pts;
    int prev_index = 0;
    int row,col;
    //最下層レイヤー描画
    //最下層レイヤーのときは、点が存在しないセルも黒色で表示する
    //上層レイヤーのときは、存在しないセルは表示しない。zを浮かして表示する。
    for(int i=0; i < pgm.layer[0].index.size(); ++i){
    // for(auto index : pgm.layer[0].index){
        //存在しないセルをポイントクラウドに追加
        // for(int i=prev_index; i < index; ++i){
        //     row = i / mapCols;
        //     col = i % mapCols;
        //     map.points[ptIndex].x = (double)((int)(mapRows/2) - row)*mapResolution - mapResolution/2;
        //     map.points[ptIndex].y = (double)(col-(int)(mapCols/2))*mapResolution + mapResolution/2;
        //     map.points[ptIndex].z = 0;
        //     map.points[ptIndex].r = 0;
        //     map.points[ptIndex].g = 0;
        //     map.points[ptIndex].b = 0;
        //     ptIndex++;
        // }
        //存在するセルをポイントクラウドに追加
        row = pgm.layer[0].index[i] / mapCols;
        col = pgm.layer[0].index[i] % mapCols;
        map.points[ptIndex].x = (double)((int)(mapRows/2) - row)*mapResolution - mapResolution/2;
        map.points[ptIndex].y = (double)(col-(int)(mapCols/2))*mapResolution + mapResolution/2;
        map.points[ptIndex].z = 0;
        colorIndex = serectColor((float)pgm.layer[0].size[i], 0.0, (float)max_cell_pts[0], colorMapGrad.size()/3)*3;
        map.points[ptIndex].r = (uint8_t)(colorMapGrad[colorIndex] * 255);
        map.points[ptIndex].g = (uint8_t)(colorMapGrad[colorIndex+1] * 255);
        map.points[ptIndex].b = (uint8_t)(colorMapGrad[colorIndex+2] * 255);
        ptIndex++;
        prev_index = pgm.layer[0].index[i] + 1;
    }
    //最後の存在しないセルを追加
    // for(int i=prev_index; i < numberOfCells; ++i){
    //         row = i / mapCols;
    //         col = i % mapCols;
    //         map.points[ptIndex].x = (double)((int)(mapRows/2) - row)*mapResolution - mapResolution/2;
    //         map.points[ptIndex].y = (double)(col-(int)(mapCols/2))*mapResolution + mapResolution/2;
    //         map.points[ptIndex].z = 0;
    //         map.points[ptIndex].r = 0;
    //         map.points[ptIndex].g = 0;
    //         map.points[ptIndex].b = 0;
    //         ptIndex++;
    // }

    //上層レイヤー描画
    for(int layer_i=1; layer_i < detection_total; ++layer_i){
        for(int i=0; i < pgm.layer[layer_i].index.size(); ++i){
        // for(auto index : pgm.layer[layer_i].index){
            //存在するセルをポイントクラウドに追加
            row = pgm.layer[layer_i].index[i] / mapCols;
            col = pgm.layer[layer_i].index[i] % mapCols;
            map.points[ptIndex].x = (double)((int)(mapRows/2) - row)*mapResolution - mapResolution/2;
            map.points[ptIndex].y = (double)(col-(int)(mapCols/2))*mapResolution + mapResolution/2;
            map.points[ptIndex].z = (double)layer_i * 0.3;
            colorIndex = serectColor((float)pgm.layer[layer_i].size[i], 0.0, (float)max_cell_pts[layer_i], colorMapGrad.size()/3)*3;
            map.points[ptIndex].r = (uint8_t)(colorMapGrad[colorIndex] * 255);
            map.points[ptIndex].g = (uint8_t)(colorMapGrad[colorIndex+1] * 255);
            map.points[ptIndex].b = (uint8_t)(colorMapGrad[colorIndex+2] * 255);
            ptIndex++;
            prev_index = pgm.layer[layer_i].size[i] + 1;
        }
    }
    map.points.resize(ptIndex);
    map.width=ptIndex;
    pcl::toROSMsg(map, map_msg);
    map_msg.header.stamp = ros::Time::now();
    map_msg.header.frame_id = "/zed_camera_center";
    gridMapPCL_pub.publish(map_msg);
}

int darknetImgDebug::serectColor(float value, float minValue, float maxValue, int palletSize){
    if(value <= minValue){
        return 0;
    }else if(maxValue <= value){
        return palletSize-1;
    }
    float range = (maxValue - minValue)/palletSize;
    return (int)((value - minValue)/range);
}

void darknetImgDebug::generateGridMapDebug(){
    pcl::PointCloud<pcl::PointXYZRGB> map;
    pcl::PointXYZRGB cell;
    sensor_msgs::PointCloud2 map_msg;
    int max_cell_pts = 0; //最大点数
    const int min_cell_pts = 0; //最小点数

    float xt, yt, zt;
    int ch = bridgeImage->image.channels();
    int rows = bridgeImage->image.rows; //深度画像の行幅
    int cols = bridgeImage->image.cols; //深度画像の列幅
    int mapRow, mapCol; // マップの行と列
    std::vector<int> map_pts;
    map_pts.resize(mapRows*mapCols);

    //処理
    for(int row = 0; row < rows; row++){
        auto bi = bridgeImage->image.ptr<float>(row);
        auto mi = mask.ptr<char>(row);
        for(int col = 0; col < cols; col++){
            if(mi[col] > 0){
                zt = bi[col*ch];
                xt = -( ((float)col-(float)cols/2)*zt/f);
                if(convertToGrid(xt, zt, mapCol, mapRow) == true){
                    map_pts[mapRow*mapCols+mapCol] ++;
                }
            }
        }
    }


   
    //初期化
    map_msg.header.stamp = smdml.header.stamp;
    map_msg.header.frame_id = "/zed_camera_center";
    map.points.resize(mapRows * mapCols);
    map.width=map.points.size();
    map.height=1;
    // int max_cell_pts_layer_index; //最大点数を持つセルの所属レイヤー（不要かも）
    // int max_cell_pts_index; //最大点数を持つセル番号（不要かも）
    //最大値算出
    for(auto cell_pts : map_pts){
        if(max_cell_pts < cell_pts){
            max_cell_pts = cell_pts;
        }
    }

    int colorIndex;
    int ptIndex;
    int cell_pts;
    for(int row = 0; row < mapRows; ++row){
        for(int col = 0; col < mapCols; ++col){
            ptIndex = row*mapCols + col;
            cell_pts = map_pts[ptIndex];
            map.points[ptIndex].x = (double)((int)(mapRows/2) - row)*mapResolution - mapResolution/2;
            map.points[ptIndex].y = (double)(col-(int)(mapCols/2))*mapResolution + mapResolution/2;
            map.points[ptIndex].z = 0;
            if(cell_pts > 0){
                colorIndex = serectColor((float)cell_pts, (float)min_cell_pts, (float)max_cell_pts, colorMapGrad.size()/3)*3;
                map.points[ptIndex].r = (uint8_t)(colorMapGrad[colorIndex] * 255);
                map.points[ptIndex].g = (uint8_t)(colorMapGrad[colorIndex+1] * 255);
                map.points[ptIndex].b = (uint8_t)(colorMapGrad[colorIndex+2] * 255);
            }else{
                map.points[ptIndex].r = 0;
                map.points[ptIndex].g = 0;
                map.points[ptIndex].b = 0;
            }
        }
    }

    pcl::toROSMsg(map, map_msg);
    map_msg.header.stamp = ros::Time::now();
    map_msg.header.frame_id = "/zed_left_camera_frame";
    gridMapPCL_pub.publish(map_msg);
}