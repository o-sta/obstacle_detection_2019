//多重インクルード防止
#ifndef INCLUDE_DETECT_PERSON_HOG
#define INCLUDE_DETECT_PERSON_HOG
//include haeders
#include <ros/ros.h>
#include <ros/callback_queue.h>
//include base class
#include <obstacle_detection_2019/darknetImg.h>
//for image processing on ros
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/opencv.hpp>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
//メッセージフィルタ
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class d_darknetImg : public darknetImg {
    private:
        //センサーデータ
        cv_bridge::CvImagePtr bridgeImage;
        //メッセージフィルタ
        message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bb_sub;
        message_filters::Subscriber<sensor_msgs::Image> image_sub;
        //sensor_callback用メッセージフィルタ
        typedef message_filters::sync_policies::ApproximateTime<darknet_ros_msgs::BoundingBoxes, sensor_msgs::Image> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync;

    /* data */
    public:
        d_darknetImg(/* args */);
        ~d_darknetImg();
        bool subscribeSensorData();//データ受信
        void sensor_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bb,const sensor_msgs::ImageConstPtr& image);
        // debug 画像と枠線を合成する
        void sensor_callback2(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bb,const sensor_msgs::ImageConstPtr& image);
        // 歩行差と背景と床面を分割する
};

#endif

