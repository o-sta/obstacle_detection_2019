#include <obstacle_detection_2019/darknetImg.h>

darknetImg::darknetImg(/* args */)
: bb_sub(nhSub, "/bb_sub", 1), image_sub(nhSub, "/image_sub", 1), sync(bb_sub, image_sub, 10)
{
    //sub = nhSub.subscribe("/zed/zed_node/left/image_rect_color",1,&darknetImg::sensor_callback,this);
    pub = nhPub.advertise<sensor_msgs::Image>("/dphog/boximage", 1);
    sync.registerCallback(boost::bind(&sensor_callback, this, _1, _2));
    //hog = cv::HOGDescriptor();
}

darknetImg::~darknetImg()
{
    
}

void darknetImg::detect()
{
    cv::cvtColor(bridgeImage->image, gray_image, CV_BGR2GRAY);
    hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
    hog.detectMultiScale(gray_image, found);
    for(int i = 0; i < found.size(); i++){
        cv::Rect r = found[i];
        rectangle(bridgeImage->image, r.tl(), r.br(), cv::Scalar(0, 255, 0), 2);
    }
}

void darknetImg::sensor_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bb, const sensor_msgs::ImageConstPtr& image)
{
    // image -> cvbridgeImage に変換する
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
    // パブリッシュ
    pub.publish(bridgeImage->toImageMsg());
}

void darknetImg::publishData()
{
    pub.publish(bridgeImage->toImageMsg());
}

