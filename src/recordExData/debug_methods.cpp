#include<obstacle_detection_2019/recordExData.h>

//デバッグ方法選択メソッド
void recordExData::debug(){
    switch(debugType){
        case 1: showMarker();break;
        default: ;
    }
}
void recordExData::showMarker(){
    //--sample
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = velClstr.header.stamp;
    marker.ns = "my_namespace";
    // marker.lifetime = ros::Duration(0.3);
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    markerArray.markers.resize((int)velClstr.data.size() * 2 + 2);
    int count = 0;
    float colors[12][3] ={{1.0,0,0},{0,1.0,0},{0,0,1.0},{1.0,1.0,0},{0,1.0,1.0},{1.0,0,1.0},{0.5,1.0,0},{0,0.5,1.0},{0.5,0,1.0},{1.0,0.5,0},{0,1.0,0.5},{1.0,0,0.5}};//色リスト
    //プリントデバッグ
    ROS_INFO("targetNum:%d", targetNum);
    if(targetNum >=0){
       ROS_INFO("estimate(x,y,z -- vx,vy):(%f,%f,%f -- %f,%f)", velClstr.data[targetNum].gc.y, velClstr.data[targetNum].gc.x, velClstr.data[targetNum].gc.z, velClstr.twist[targetNum].linear.x, velClstr.twist[targetNum].linear.y);
    }
    ROS_INFO("turtle(x,y,z -- vx,vy):(%f,%f,%f -- %f,%f)", turtle_odom.pose.pose.position.x, turtle_odom.pose.pose.position.y, turtle_odom.pose.pose.position.z, turtle_odom.twist.twist.linear.x, turtle_odom.twist.twist.linear.y);
    ROS_INFO("measure(x,y,z -- vx,vy):(%f,%f,%f -- %f,%f)", measure_odom.pose.pose.position.x, measure_odom.pose.pose.position.y, measure_odom.pose.pose.position.z, measure_odom.twist.twist.linear.x, measure_odom.twist.twist.linear.y);
    //
    for(int k=0; k<velClstr.data.size(); k++){
        //
        if(k == targetNum){//
            marker.scale.x = 1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.text = "("+std::to_string(turtle_odom.twist.twist.linear.y) +","+ std::to_string(turtle_odom.twist.twist.linear.x)+")";
            // marker.text = "("+std::to_string(turtle_odom.twist.twist.linear.x) +","+ std::to_string(turtle_odom.twist.twist.linear.y)+")";
            //pose
            marker.pose = turtle_odom.pose.pose;
            //color
            marker.color.a = 1.0;
            marker.color.r = colors[k][0];
            marker.color.g = colors[k][1];
            marker.color.b = colors[k][2];
            //add Array
            //--arrow
            marker.type = visualization_msgs::Marker::ARROW;
            marker.id = count;
            markerArray.markers[count++] = marker;
            //--text
            marker.scale.x = 1;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.id = count;
            markerArray.markers[count++] = marker;
        }
        //
        marker.scale.x = 1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.text = "("+std::to_string(velClstr.twist[k].linear.x) +","+ std::to_string(velClstr.twist[k].linear.y)+")";
        //position
        marker.pose.position.x = velClstr.data[k].gc.y;
        marker.pose.position.y = velClstr.data[k].gc.x;
        marker.pose.position.z = velClstr.data[k].gc.z + 1.0;
        //angle
        double yaw = std::atan2(velClstr.twist[k].linear.x,velClstr.twist[k].linear.y);
        //culc Quaternion
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        marker.color.a = 1.0;
        marker.color.r = colors[k][0];
        marker.color.g = colors[k][1];
        marker.color.b = colors[k][2];

        //--arrorw
        marker.type = visualization_msgs::Marker::ARROW;
        //add Array
        marker.id = count;
        markerArray.markers[count++] = marker;
        //--text
        marker.scale.x = 1;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.id = count;
        markerArray.markers[count++] = marker;
    }
    markerArray.markers.resize(count);
    // ROS_INFO("markerArray.markers.size():%d",(int)markerArray.markers.size());
    if(markerArray.markers.size()){
        pubDebMarker.publish( markerArray );
    }

}
