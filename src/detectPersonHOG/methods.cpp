#include <obstacle_detection_2019/detectPersonHOG.h>

detectPersonHOG::detectPersonHOG(/* args */)
{
    sub = nhSub.subscribe("/zed/zed_node/left/image_rect_color",1,&detectPersonHOG::sensor_callback,this);
    pub = nhPub.advertise<sensor_msgs::Image>("/dphog/boximage", 1);
    //hog = cv::HOGDescriptor();
}

detectPersonHOG::~detectPersonHOG()
{
    
}

void detectPersonHOG::detect()
{
    cv::cvtColor(bridgeImage->image, gray_image, CV_BGR2GRAY);
    hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
    hog.detectMultiScale(gray_image, found);
    for(int i = 0; i < found.size(); i++){
        cv::Rect r = found[i];
        rectangle(bridgeImage->image, r.tl(), r.br(), cv::Scalar(0, 255, 0), 2);
    }
}

void detectPersonHOG::sensor_callback(const sensor_msgs::ImageConstPtr& msg)
{
    try{
        bridgeImage=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::RGB8);
    }
    catch(cv_bridge::Exception& e) {//エラー処理
        ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.",msg->encoding.c_str());
        return;
    }
    detect();
    publishData();
}

void detectPersonHOG::publishData()
{
    pub.publish(bridgeImage->toImageMsg());
}

