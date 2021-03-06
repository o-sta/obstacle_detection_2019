#include<obstacle_detection_2019/synchroImage.h>

syncroImageClass::syncroImageClass()
	:rgb_sub(nhSub, "zed_node/left/image_rect_color", 1),depth_sub(nhSub, "zed_node/depth/depth_registered", 1)//,sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>(10), rgb_sub, depth_sub)
    ,sync(MySyncPolicy(10),rgb_sub, depth_sub),debugType(0)
{
    //subscriber
    sync.registerCallback(boost::bind(&syncroImageClass::callback, this,_1, _2));
    pub= nhPub.advertise<obstacle_detection_2019::synchronizedImage>("syncronized_image", 1);
	pubDebPcl = nhDeb.advertise<sensor_msgs::PointCloud2>("synchroPoints", 1);
}
syncroImageClass::~syncroImageClass(){

}

