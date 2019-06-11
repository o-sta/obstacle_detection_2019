#include <ros/ros.h>
#include <obstacle_detection_2019/convCamData.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>

class d_convCamDataClass : public convCamDataClass{
    private:
        sensor_msgs::PointCloud2Ptr d_point_msg; //パブリッシュ用ポイントクラウド
        ros::NodeHandle d_nhPub1;
        ros::Publisher d_pubPcl;
    public:
        d_convCamDataClass()
        :d_point_msg(new sensor_msgs::PointCloud2)
        {
            d_pubPcl = d_nhPub1.advertise<sensor_msgs::PointCloud2>("d_ground_pointcloud", 1);
        }
        ~d_convCamDataClass(){
        }
        // SensorMapData を sensor_msgs::PointCloud2 に変換
        void d_smd2pcl(){
            d_point_msg->header.stamp = ros::Time::now();
            d_point_msg->header = smd.header;
            d_point_msg->height = smd.heightInt.data;
            d_point_msg->width = smd.widthInt.data;
            ROS_INFO_STREAM(smd.heightInt.data << " : " << smd.widthInt.data);
            d_point_msg->is_bigendian = false;
            d_point_msg->point_step = 16;
            d_point_msg->row_step = d_point_msg->point_step * d_point_msg->width;
            d_point_msg->is_dense = false;
            sensor_msgs::PointCloud2Modifier pcl_mod(*d_point_msg);
            pcl_mod.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
                                            "y", 1, sensor_msgs::PointField::FLOAT32,
                                            "z", 1, sensor_msgs::PointField::FLOAT32,
                                            "rgb", 1, sensor_msgs::PointField::FLOAT32);
            sensor_msgs::PointCloud2Iterator<float> iter_x(*d_point_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(*d_point_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(*d_point_msg, "z");
            sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(*d_point_msg, "rgb");
            auto iter_smd = smd.pt.begin();
            auto iter_index = smd.index.begin();
            for(; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_rgb, ++iter_smd, ++iter_index){
                *iter_x = iter_smd->x;
                *iter_y = iter_smd->y;
                *iter_z = iter_smd->z;
                iter_index->data == -1 ? iter_rgb[0] = 255 : iter_rgb[0] = 0;
                iter_index->data == -1 ? iter_rgb[1] = 255 : iter_rgb[1] = 0;
                iter_rgb[2] = 255;
            }
        }
        // pcl をパブリッシュ
        void d_publish_pcl(){
            d_pubPcl.publish(d_point_msg);
        }
        // MaskImageData を sensor_msgs::
        void d_publish_pcl(){
            
        }
};
