#include <obstacle_detection_2019/darknetImgDebug.h>

void darknetImgDebug::publishGridMap(){
    pcl::PointCloud<pcl::PointXYZRGB> map;
    pcl::PointXYZRGB cell;
    sensor_msgs::PointCloud2 map_msg;
    int max_cell_pts = 0; //最大点数
    const int min_cell_pts = 0; //最小点数
    //初期化
    map_msg.header.stamp = smdml.header.stamp;
    map_msg.header.frame_id = "/zed_camera_center";
    map.points.resize(mapRows * mapCols);
    map.width=map.points.size();
    map.height=1;
    // int max_cell_pts_layer_index; //最大点数を持つセルの所属レイヤー（不要かも）
    // int max_cell_pts_index; //最大点数を持つセル番号（不要かも）
    //最大値算出
    for(auto layer : smdml.layer){
        for(auto cell_index : layer.index){
            if(cell_index.data >= 0){
                if(layer.size[cell_index.data].data > max_cell_pts){
                    max_cell_pts = layer.size[cell_index.data].data;
                    // max_cell_pts_index = cell_index.data;
                }
            }
        }
    }
    int colorIndex;
    int ptIndex;
    double gauss;
    int cell_pts;
    int index;
    for(int row = 0; row < mapRows; ++row){
        for(int col = 0; col < mapCols; ++col){
            ptIndex = row*mapCols + col;
            cell_pts = 0;
            for(auto layer : smdml.layer){
                index = layer.index[ptIndex].data;
                if(index >= 0){
                    cell_pts += layer.size[index].data;
                }
            }
            map.points[ptIndex].x = (double)((int)(mapRows/2) - row)*mapResolution - mapResolution/2;
            map.points[ptIndex].y = (double)(col-(int)(mapCols/2))*mapResolution + mapResolution/2;
            map.points[ptIndex].z = 0;
            colorIndex = serectColor((float)cell_pts, (float)min_cell_pts, (float)max_cell_pts, colorMapGrad.size()/3)*3;
            map.points[ptIndex].r = (uint8_t)(colorMap[colorIndex] * 255);
            map.points[ptIndex].g = (uint8_t)(colorMap[colorIndex+1] * 255);
            map.points[ptIndex].b = (uint8_t)(colorMap[colorIndex+2] * 255);
        }
    }

    pcl::toROSMsg(map, map_msg);
    map_msg.header.stamp = ros::Time::now();
    map_msg.header.frame_id = "/map";
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