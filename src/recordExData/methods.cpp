#include<obstacle_detection_2019/recordExData.h>

//callback 
void recordExData::velCluster_callback(const obstacle_detection_2019::ClassificationVelocityData::ConstPtr& msg){
    velClstr = *msg;
    timeNow = ros::Time::now();
    ROS_INFO("velCluster_callback");
    //move manage 
    manage();
}
void recordExData::odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
    measure_odom = *msg;
    timeNow = ros::Time::now();
    ROS_INFO("odom_callback");
    //move manage 
    manage();
}
//manage
void recordExData::manage() {
    ROS_INFO("transObstaclePos");
    transObstaclePos();
    ROS_INFO("targetDetect");
    targetDetect();
    ROS_INFO("setEstimateData");
    setEstimateData();
    ROS_INFO("debug");
    debug();
    if(targetNum>=0){//障害物が検出された時
        setRecordData();
        publishRecordData();
        if(outputCSV){
            ROS_INFO("outputCSVData");
            outputCSVData();
        }
    }
}
//processes
// defined
// nav_msgs::Odometry measure_odom, estimate_odom;//measure_odom=/robot1/odom, estimate_odom=estimateData
// nav_msgs::Odometry turtle_odom, beego_odom;

//transform geometry_msgs::Point to tf::Vector3
tf::Vector3 recordExData::transPointToVector3(geometry_msgs::Point& pt){
    return tf::Vector3(pt.x, pt.y, pt.z);
}
//transform tf::Vector3 to geometry_msgs::Point
void recordExData::transVector3ToPoint(const tf::Vector3& pt, geometry_msgs::Point& bt){
    geometry_msgs::Vector3 vec3;
    tf::vector3TFToMsg(pt,vec3);
    bt.x = vec3.x;
    bt.y = vec3.y;
    bt.z = vec3.z;
}

//障害物の位置を障害物座標系ー＞ビーゴ座標系に変換するメソッド
//measure_odom + init_turtle_odom -> turtle_odom
void recordExData::transObstaclePos() {
    //
    // 変換行列を構築する
    tf::Transform transformer;
    tf::Quaternion init_turtle_quat;
    tf::quaternionMsgToTF(init_turtle_odom.pose.pose.orientation, init_turtle_quat);
    transformer.setOrigin(transPointToVector3(init_turtle_odom.pose.pose.position));
    transformer.setRotation(init_turtle_quat);
    // 速度は向きだけ変換
    tf::Transform velocity_transformer;
    velocity_transformer.setRotation(init_turtle_quat);

    // 座標変換を行う
    //位置
    tf::Vector3 turtlePosition = transPointToVector3(measure_odom.pose.pose.position);
    tf::Vector3 turtlePositionTrans = transformer * turtlePosition;
    tf::Quaternion measure_quat;
    tf::quaternionMsgToTF(measure_odom.pose.pose.orientation, measure_quat);
    tf::Quaternion turtleQuat = transformer * measure_quat;
    //速度
    tf::Vector3 turtleVelocity;
    tf::vector3MsgToTF(measure_odom.twist.twist.linear,turtleVelocity);
    tf::Vector3 turtleVelocityTrans = velocity_transformer * turtleVelocity;

    //変換後
    transVector3ToPoint(turtlePositionTrans, turtle_odom.pose.pose.position);
    tf::quaternionTFToMsg(turtleQuat, turtle_odom.pose.pose.orientation);
    tf::vector3TFToMsg(turtleVelocityTrans,turtle_odom.twist.twist.linear);
    //他データの格納
    //--twist angular
    turtle_odom.twist.twist.angular = measure_odom.twist.twist.angular;//変換無し
    //--header
    turtle_odom.header.seq += 1;
    turtle_odom.header.stamp = timeNow;
    
}
//対象の障害物番号を探索するメソッド
//対象の番号をtargetNumに格納
//タートルボットの自己位置（turtle_odom）と速度推定結果（各クラスタの重心位置）から対象を探索する
void recordExData::targetDetect() {
    // int targetNum;
    //位置の差
    geometry_msgs::Point difPoint;
    double difDis;
    double minDis = 100;
    int minNum = -1;
    for(int k=0; k<velClstr.data.size(); k++){
        //プログラム処理の軸(Xp,Yp, Zp)とROSの軸(Xr, Yr, Zr): Xp = Yr, Yp = Xr Zp = Zr
        difPoint.x = velClstr.data[k].gc.y - turtle_odom.pose.pose.position.x;//dif x
        difPoint.y = velClstr.data[k].gc.x - turtle_odom.pose.pose.position.y;//dif y
        difPoint.z = 0;
        difDis = std::sqrt(difPoint.x * difPoint.x + difPoint.y * difPoint.y + difPoint.z * difPoint.z);
        if(difDis > distanceThreshold){
            continue;
        }
        if(minNum < 0 || difDis < minDis){
            minNum = k;
        }
    }
    targetNum = minNum;
}
//推定データを格納
void recordExData::setEstimateData() {
    //position
    estimate_odom.pose.pose.position.x = velClstr.data[targetNum].gc.y;
    estimate_odom.pose.pose.position.y = velClstr.data[targetNum].gc.x;
    estimate_odom.pose.pose.position.z = velClstr.data[targetNum].gc.z; 
    //angle
    double yaw = std::atan2(velClstr.twist[targetNum].linear.x,velClstr.twist[targetNum].linear.y);
    //to Quaternion
    estimate_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    //linear
    estimate_odom.twist.twist.linear.x = velClstr.twist[targetNum].linear.y;
    estimate_odom.twist.twist.linear.y = velClstr.twist[targetNum].linear.x;
    estimate_odom.twist.twist.linear.z = velClstr.twist[targetNum].linear.z;
    //angular
    estimate_odom.twist.twist.angular.x = velClstr.twist[targetNum].angular.y;
    estimate_odom.twist.twist.angular.y = velClstr.twist[targetNum].angular.x;
    estimate_odom.twist.twist.angular.z = velClstr.twist[targetNum].angular.z;
    //
    //--header
    estimate_odom.header.seq += 1;
    estimate_odom.header.stamp = timeNow;
}
void recordExData::setRecordData(){
    // obstacle_detection_2019::recordData recordData;    
    // time
    recordData.stamp = timeNow;
    // #pose
    recordData.estimateX = velClstr.data[targetNum].gc.y;
    recordData.estimateY = velClstr.data[targetNum].gc.x;
    recordData.estimateYaw = std::atan2(velClstr.twist[targetNum].linear.x,velClstr.twist[targetNum].linear.y);
    recordData.obstacleX = turtle_odom.pose.pose.position.x;
    recordData.obstacleY = turtle_odom.pose.pose.position.y;
    recordData.obstacleYaw = std::atan2(turtle_odom.twist.twist.linear.y, turtle_odom.twist.twist.linear.x);
    // #vel
    recordData.estimateVx = velClstr.twist[targetNum].linear.y;
    recordData.estimateVy = velClstr.twist[targetNum].linear.x;
    recordData.obstacleVx = turtle_odom.twist.twist.linear.x;
    recordData.obstacleVy = turtle_odom.twist.twist.linear.y;
    // #dif vel: estimate - encorder
    recordData.difVx = recordData.estimateVx - recordData.obstacleVx;
    recordData.difVy = recordData.estimateVy - recordData.obstacleVy;
}
void recordExData::publishRecordData(){
    pub.publish(recordData);
}
void recordExData::setCSVDataLabel(){
    std::ofstream ofs(userpath+filepath+filename+".csv",std::ios::app);
    ofs<<"stamp"<<","
        <<"estimateX"<<","
        <<"estimateY"<<","
        <<"estimateYaw"<<","
        <<"obstacleX"<<","
        <<"obstacleY"<<","
        <<"obstacleYaw"<<","
        <<"estimateVx"<<","
        <<"estimateVy"<<","
        <<"obstacleVx"<<","
        <<"obstacleVy"<<","
        <<"difVx"<<","
        <<"difVy"<<","
        <<std::endl;
}
void recordExData::outputCSVData(){
    std::ofstream ofs(userpath+filepath+filename+".csv",std::ios::app);
    ofs<<recordData.stamp<<","
        <<recordData.estimateX<<","
        <<recordData.estimateY<<","
        <<recordData.estimateYaw<<","
        <<recordData.obstacleX<<","
        <<recordData.obstacleY<<","
        <<recordData.obstacleYaw<<","
        <<recordData.estimateVx<<","
        <<recordData.estimateVy<<","
        <<recordData.obstacleVx<<","
        <<recordData.obstacleVy<<","
        <<recordData.difVx<<","
        <<recordData.difVy<<","
        <<std::endl;
}