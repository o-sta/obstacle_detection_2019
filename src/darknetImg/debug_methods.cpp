#include <obstacle_detection_2019/darknetImgDebug.h>



// void darknetImg::sensor_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bb, const sensor_msgs::Image::ConstPtr& image)
// {
//     // debug移行予定
//     ROS_INFO_STREAM("callback : " << "[bb : " << bb->header.stamp.toSec() << " ]," 
//                                   << "[img : " << image->header.stamp.toSec() << " ],"
//                                   << "[diff : " << (bb->header.stamp - image->header.stamp).toSec() << "]");
//     // image から cvbridgeImage に変換する
//     try{
//         bridgeImage=cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);
//     }
//     catch(cv_bridge::Exception& e) {//エラー処理
//         ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.",image->encoding.c_str());
//         return;
//     }
//     // bounding_boxedに書かれた枠の描画
//     auto iter = bb->bounding_boxes.begin();
//     for (; iter != bb->bounding_boxes.end(); ++iter){
//         cv::rectangle(bridgeImage->image, cv::Point(iter->xmin, iter->ymin), cv::Point(iter->xmax, iter->ymax), cv::Scalar(0, 0, 200), 5, 8);
//     }
//     // パブリッシュ(debgu移行予定)
//     pub.publish(bridgeImage->toImageMsg());
// }