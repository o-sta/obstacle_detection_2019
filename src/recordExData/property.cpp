#include<obstacle_detection_2019/recordExData.h>

void recordExData::setLaunchParam(){
    
    // nav_msgs::Odometry init_turtle_odom, init_beego_odom;
    //init Header
    std_msgs::Header turtle_hd, beego_hd;
    ros::Time timeInit = ros::Time::now();
    turtle_hd.seq = 0;
    turtle_hd.stamp = timeInit;
    turtle_hd.frame_id = "odom";//グローバル座標系
    beego_hd.seq = 0;
    beego_hd.stamp = timeInit;
    beego_hd.frame_id = "odom";//グローバル座標系
    // --set
    init_turtle_odom.header = turtle_hd;
    init_beego_odom.header = beego_hd;
    init_turtle_odom.child_frame_id = "base_link";//ビーゴの座標系に固定
    init_beego_odom.child_frame_id = "base_link";//ビーゴの座標系に固定
    // init param
    // pos
    //beego
    init_beego_odom.pose.pose.position.x = 0;//x
    init_beego_odom.pose.pose.position.y = 0;//y
    init_beego_odom.pose.pose.position.z = 0;//z
    init_beego_yaw = 0;
    //turtle
    init_turtle_odom.pose.pose.position.x = 0;//x
    init_turtle_odom.pose.pose.position.y = 0;//y
    init_turtle_odom.pose.pose.position.z = 0;//z
    init_turtle_yaw = M_PI/2.0;
    // twist
    //beego
    init_beego_odom.twist.twist.linear.x = 0;//x
    init_beego_odom.twist.twist.linear.y = 0;//y
    init_beego_odom.twist.twist.linear.z = 0;//z
    init_beego_odom.twist.twist.angular.x = 0;//x
    init_beego_odom.twist.twist.angular.y = 0;//y
    init_beego_odom.twist.twist.angular.z = 0;//z
    //turtle
    init_turtle_odom.twist.twist.linear.x = 0;//x
    init_turtle_odom.twist.twist.linear.y = 0;//y
    init_turtle_odom.twist.twist.linear.z = 0;//z
    init_turtle_odom.twist.twist.angular.x = 0;//x
    init_turtle_odom.twist.twist.angular.y = 0;//y
    init_turtle_odom.twist.twist.angular.z = 0;//z

    //set launch param
    ros::NodeHandle n("~");
    //rqt_reconfigure
    n.getParam("recordExData/rqt_reconfigure", rqt_reconfigure);
    //ロボットの台数(仮)
    n.getParam("recordExData/robotNum", robotNum);
    //ロボット（回避）の初期位置
    // double init_turtle_yaw, init_beego_yaw;//defined
    n.getParam("recordExData/robot2/x", init_beego_odom.pose.pose.position.x);//x
    n.getParam("recordExData/robot2/y", init_beego_odom.pose.pose.position.y);//y
    n.getParam("recordExData/robot2/yaw", init_beego_yaw);//yaw
    init_beego_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(init_beego_yaw);//yaw->quat
    //ロボット（障害物）の初期位置
    n.getParam("recordExData/robot1/x", init_turtle_odom.pose.pose.position.x);//x
    n.getParam("recordExData/robot1/y", init_turtle_odom.pose.pose.position.y);//y
    n.getParam("recordExData/robot1/yaw", init_turtle_yaw);//yaw
    init_turtle_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(init_turtle_yaw);//yaw->quat
    //ロボット（回避）の速度
    n.getParam("recordExData/robot2/v", init_beego_odom.twist.twist.linear.x);//x
    n.getParam("recordExData/robot2/w", init_beego_odom.twist.twist.angular.z);//z
    //ロボット（障害物）の速度
    n.getParam("recordExData/robot1/v", init_turtle_odom.twist.twist.linear.x);//x
    n.getParam("recordExData/robot1/w", init_turtle_odom.twist.twist.angular.z);//z
    //タートルボット検出距離の閾値
    n.getParam("recordExData/distanceThreshold", distanceThreshold);//[m]
    //デバッグ
    n.getParam("recordExData/debugType",debugType);
}
void recordExData::configCallback(obstacle_detection_2019::recordExDataConfig &config, uint32_t level) {
	ROS_INFO_STREAM("Reconfigure Request" << std::endl
        << "robotNum: "<< config.robotNum << std::endl
        << "robotPos: "<< config.robotPosX << "," << config.robotPosY << "," << config.robotYaw << std::endl 
        << "obstaclePos: "<< config.obstaclePosX << "," << config.obstaclePosY << "," << config.obstacleYaw << std::endl 
        << "robotVel: "<< config.robotVel << "," << config.robotAngularVel << std::endl 
        << "obstacleVel: "<< config.obstacleVel << "," << config.obstacleAngularVel << std::endl 
        << "distanceThreshold: "<< config.distanceThreshold << std::endl 
        << "debugType: "<< config.debugType << std::endl
		);
    robotNum = config.robotNum;
    init_beego_odom.pose.pose.position.x = config.robotPosX;
    init_beego_odom.pose.pose.position.y = config.robotPosY;
    init_beego_yaw = config.robotYaw;
    init_beego_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(init_beego_yaw);//yaw->quat
    init_turtle_odom.pose.pose.position.x = config.obstaclePosX;
    init_turtle_odom.pose.pose.position.y = config.obstaclePosY;
    init_turtle_yaw = config.obstacleYaw;
    init_turtle_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(init_turtle_yaw);//yaw->quat
    init_beego_odom.twist.twist.linear.x = config.robotVel;
    init_beego_odom.twist.twist.angular.z = config.robotAngularVel;
    init_turtle_odom.twist.twist.linear.x = config.obstacleVel;
    init_turtle_odom.twist.twist.angular.z = config.obstacleAngularVel;
    distanceThreshold = config.distanceThreshold;
    debugType = config.debugType;
    outputCSV = config.outputCSV;
    filename = config.fileName;
	//csv データラベル
	if(outputCSV){
		setCSVDataLabel();
	}
}
