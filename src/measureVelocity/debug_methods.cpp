#include<obstacle_detection_2019/measurementVelocity.h>

//デバッグ方法選択メソッド
void measurementVelocity::debug(){
    switch(debugType){
        case 1: showPointcloud();break;
        case 2: showMarker();break;
        default: ;
    }
}

//計測した速度データをポイントクラウドで可視化
// 1,2,3,,,秒後の点群データを表示し、障害物の速度ベクトルを表示
// rviz上での矢印、速度の文字表示を行いたいが、調べる時間が惜しいため
// 現在は未実装
void measurementVelocity::showPointcloud(){
    //障害物カラーレパートリー
	float colors[12][3] ={{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127}};//色リスト
    //表示用ポイントクラウド
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr viewCloud(new  pcl::PointCloud<pcl::PointXYZRGB>);
    //初期化
    //データ数の計算
    int pointNum = 0;//総データ数
    for(int i = 0; i < cvd.data.size(); i++){
        if(trackNum[i] < trackThreshold ){
            continue;
        }
        pointNum += (int)cvd.data[i].pt.size();
    }
    //
    //表示時間設定
    // float timeRange = 10;//表示時間範囲(~秒後まで表示)
    // float timeInteval = 1;//表示時間間隔(~秒ごとに表示)
    int timeLoop = (int)(timeRange/timeInteval) + 1;//ループ回数(時間)
    //表示高さ幅設定(点１つでは見えにくいため表示数を増やす)
    //表示範囲, 表示個数
    //z+zUnder <= z <= z+zUpper
    //(zUpper - zUnder)/zDelta
    float zUpper =0.5;
    float zUnder = -0.5;
    float zDelta =0.1;
    int zLoop = (int)((zUpper - zUnder)/zDelta) + 1;//ループ回数(z軸表示幅)
    //データ数再計算
    pointNum = pointNum * timeLoop * zLoop;
	viewCloud->points.clear();
	viewCloud->points.resize(pointNum);
    
    //要素追加用の仮変数
	pcl::PointXYZRGB cloudTemp;
    //追加済み要素数カウント
    int count = 0;
    //pointCloudデータ作成
    viewCloud->width = count;
    viewCloud->height = 1;
    for(float t = 0; t <= timeRange; t+=timeInteval){
        // ROS_INFO_STREAM(t<<","<<timeRange<<","<<timeInteval);
        //各クラスタごとの処理
        for(int i = 0; i < cvd.data.size(); i++){
            // if(cvd.twist[i].linear.y*cvd.twist[i].linear.y + cvd.twist[i].linear.x*cvd.twist[i].linear.x > 0 ){
            //     ROS_INFO_STREAM(cvd.twist[i]);
            // }
            if(trackNum[i] < trackThreshold ){
                continue;
            }
            //非表示処理
            //--データ数が閾値以下の時
            // if(cvd.data[i].size.data < 10){
            //     continue;
            // }
            //カラー設定
			cloudTemp.r=colors[i%12][0];
			cloudTemp.g=colors[i%12][1];
			cloudTemp.b=colors[i%12][2];
            //各データごとの処理
            for(int k = 0; k < cvd.data[i].pt.size(); k++){  
    			cloudTemp.x=cvd.data[i].pt[k].y;//y軸              
                cloudTemp.y=cvd.data[i].pt[k].x;//逆向きのx軸
                //時間経過分, 移動
                cloudTemp.x += cvd.twist[i].linear.y * t;//y軸 
                cloudTemp.y += cvd.twist[i].linear.x * t;//逆向きのx軸      
                //表示幅分点を追加
                for(int n=0; n<= (int)((zUpper - zUnder)/zDelta); n++){
                    cloudTemp.z=cvd.data[i].pt[k].z + zUnder + n*zDelta;//z軸 + 表示範囲              
                    //ポイントクラウドに追加
                    viewCloud->points[count++] = cloudTemp;
                    viewCloud->width = count;
                }
            }
        }
    }
	
	std::cout<<"viewCloud->points.size():"<<viewCloud->points.size()<<"\n";
    //データがないとき
	if(viewCloud->width <= 0)
	{
        ROS_INFO("No point cloud data!");
		return ;
	}
	sensor_msgs::PointCloud2 viewMsgs;
	pcl::toROSMsg (*viewCloud, viewMsgs);
	viewMsgs.header.frame_id="/zed_camera_center";
	pubDeb.publish(viewMsgs);

}

void measurementVelocity::showMarker(){

}
