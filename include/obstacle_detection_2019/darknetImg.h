//多重インクルード防止
#ifndef INCLUDE_DETECT_PERSON_HOG
#define INCLUDE_DETECT_PERSON_HOG
//include haeders
#include <ros/ros.h>
#include <ros/callback_queue.h>
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
//shared ptr
//#include <boost/shared_ptr.hpp>
//#include <boost/make_shared.hpp>

class darknetImg {
    private:
        //センサーデータ
		ros::NodeHandle nhSub;
		ros::Subscriber sub;
		//ros::CallbackQueue queue;
        cv_bridge::CvImagePtr bridgeImage;
        //送信データ0
		ros::NodeHandle nhPub;
        ros::Publisher pub;
        //使用データ
        std::vector<cv::Rect> found;
        cv::Mat gray_image;
        cv::Mat image;
        message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bb_sub;
        message_filters::Subscriber<sensor_msgs::Image> image_sub;
        //message_filters::TimeSynchronizer<darknet_ros_msgs::BoundingBoxes, sensor_msgs::Image> sync;
        typedef message_filters::sync_policies::ApproximateTime<darknet_ros_msgs::BoundingBoxes, sensor_msgs::Image> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync;

    /* data */
    public:
        darknetImg(/* args */);
        ~darknetImg();
        void detect(); //人間を検出
        bool subscribeSensorData();//データ受信
        void sensor_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bb,const sensor_msgs::ImageConstPtr& image); //データ受信
        void publishData(); //データ送信
        // message_filters::Subscriber<darknet_ros_ms>
        cv::HOGDescriptor hog;
};

#endif

