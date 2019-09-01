#include <obstacle_detection_2019/darknetImg.h>

// sensor_callback用コンストラクタ
// darknetImg::darknetImg(/* args */)
// : bb_sub(nhSub, "/darknet_ros/bounding_boxes", 1), image_sub(nhSub, "/robot2/zed_node/left/image_rect_color", 1), sync(MySyncPolicy(10),bb_sub, image_sub)
// {
//     pub = nhPub.advertise<sensor_msgs::Image>("/dphog/boximage", 1);
//     sync.registerCallback(boost::bind(&darknetImg::sensor_callback, this, _1, _2));
// }

// sensor_callback2用コンストラクタ
darknetImg::darknetImg(/* args */)
: 
bb_sub(nhSub, "/darknet_ros/bounding_boxes", 1), 
image_sub(nhSub, "/robot2/zed_node/left/image_rect_color", 1), 
sync(MySyncPolicy(10),bb_sub, image_sub),
is_size_initialized(false),
mask(1,1,CV_8UC1),
ground_points(new pcl::PointCloud<pcl::PointXYZ>)
{
    pub = nhPub.advertise<sensor_msgs::Image>("/dphog/boximage", 1);
    sync.registerCallback(boost::bind(&darknetImg::sensor_callback, this, _1, _2));
    mapWidth = 8;
    mapHeight = 8;
    mapResolution = 0.05;
    mapRows = (int)(mapHeight/mapResolution);
    mapCols = (int)(mapWidth/mapResolution);
    numberOfCells = mapRows*mapCols;
}

darknetImg::~darknetImg()
{
    
}

void darknetImg::sensor_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bb, const sensor_msgs::Image::ConstPtr& image)
{
    // debug移行予定
    ROS_INFO_STREAM("callback : " << "[bb : " << bb->header.stamp.toSec() << " ]," 
                                  << "[img : " << image->header.stamp.toSec() << " ],"
                                  << "[diff : " << (bb->header.stamp - image->header.stamp).toSec() << "]");
    // image から cvbridgeImage に変換する
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
    // パブリッシュ(debgu移行予定)
    pub.publish(bridgeImage->toImageMsg());
}

void darknetImg::pickUpGroundPointCandidates(){
    float xt, yt, zt; //点の座標（テンプレート）
    pcl::PointXYZ pt; //点の座標（ポイントクラウド型テンプレート）
    int rows = bridgeImage->image.rows; //深度画像の行
    int cols = bridgeImage->image.cols; //深度画像の列
    int candidateNum = 0;//床面候補点の数(ground_pointsのサイズ)
    ground_points->points.resize(rows/2*cols);//候補点の最大値でリサイズ
    int ch = bridgeImage->image.channels(); //チャンネル数
    //床面候補点抽出処理
    for(int i = rows/2+1; i<rows; i++){//画像下半分を走査
        float *p = bridgeImage->image.ptr<float>(i); //i行1列目のアドレス
        for(int j = 0; j < cols; j++){//走査}
            zt = p[j*ch];
            if(zt > 0.5 && !std::isinf(zt)){
                yt = ((float)rows/2-i)*zt/f; //高さ
                if(std::abs(yt+camHeight) < groundCandidateY){ //高さがgroundCandidateY未満の時
                    xt = -( ((float)i-(float)cols/2)*zt/f-camHeight );
                    pt.x=zt;
                    pt.y=xt;
                    pt.z=yt;
                    ground_points->points[candidateNum++] = pt;
                }
            }
        }
    }
    ground_points->points.resize(candidateNum);
    ground_points->width=ground_points->points.size();
    ground_points->height=1;
    ROS_INFO_STREAM("ground_points->points.size():"<<ground_points->points.size()<<"\n");
}

void darknetImg::estimateGroundCoefficients(){
    seg.setInputCloud(ground_points);
	seg.segment(*inliers, *coefficients);
    a=coefficients->values[0];
    b=coefficients->values[1];
    c=coefficients->values[2];
    d=coefficients->values[3];
    //床面式の確認と再設定
	//const float ground_th=0.20;//床面式から高さground_thまでのデータを床面とする
    if(std::abs(d-camHeight>=0.15)){//推定値があまりにも変な場合
        //事前に決めた値を使用
		a=-0.08;
		b=0;
		c=1;
		d=camHeight-ground_th;
	}
	else{
		d-=ground_th;
	}
    ROS_INFO_STREAM("Model coefficients: " << a << " "
                                    << b << " "
                                    << c << " "
                                    << d << "\n");
}

void darknetImg::removeGroundPoints(){
    int row = 0, col = 0;
    float xt, yt, zt;
    int ch = bridgeImage->image.channels();
    int rows = bridgeImage->image.rows; //深度画像の行
    int cols = bridgeImage->image.cols; //深度画像の列
    //マスクのりサイズ
    if(is_size_initialized == false){
        cv::resize(mask, mask, cv::Size(rows,cols));
    }
    for(row = 0; row < rows; row++){
        float *bi = bridgeImage->image.ptr<float>(row);
        char *mi = mask.ptr<char>(row);
        for(col = 0; col < cols; col++){
            zt = bi[col*ch];
            if(zt>0.5&&!std::isinf(zt)){
                yt=((float)cols/2-row)*zt/f;//高さ算出
                //xt=-( ((float)row-(float)rows/2)*zt/f-camHeight );
                float y_ground=(-a*zt-b*xt-d)/c;//床面の高さを算出
                //高さが床面以上であればmask値を1に
                yt > y_ground ? mi[col] = 1 : mi[col] = 0;
            } else{
                mi[col] = 0;
            }
        }
    }
}

void darknetImg::trimPoints(){
    int i = 0;
    for(const auto& bb : boundingBoxesMsg.bounding_boxes){
        cv::rectangle(mask, cv::Point(bb.xmin, bb.ymin), cv::Point(bb.xmax, bb.ymax), cv::Scalar(++i), -1, CV_FILLED);
    }
}

void darknetImg::generateGridmap(){
    int row = 0, col = 0, i; //深度画像の行と列
    float xt, yt, zt;
    int ch = bridgeImage->image.channels();
    int rows = bridgeImage->image.rows; //深度画像の行幅
    int cols = bridgeImage->image.cols; //深度画像の列幅
    geometry_msgs::Point pt;
    pt.x = 0; pt.y = 0; pt.z = 0;
    int mapRow, mapCol; // マップの行と列
    //初期設定
    smdml.layer.resize(boundingBoxesMsg.bounding_boxes.size());
    for(auto& layer : smdml.layer){
        layer.index.resize(rows*cols);
        layer.header = smdml.header;
        layer.width.data = mapWidth;
        layer.height.data = mapHeight;
        layer.res.data = mapResolution;
        layer.widthInt.data = mapCols;
        layer.heightInt.data = mapRows;
        layer.index.resize(numberOfCells);
        layer.size.resize(numberOfCells);
        layer.pt.resize(numberOfCells);
        std::fill(layer.index.begin(), layer.index.end(), -1); //std::vectorの全要素を-1で埋める
        std::fill(layer.size.begin(), layer.size.end(), 0);
        std::fill(layer.pt.begin(), layer.pt.end(), pt);
    }
    //処理
    int count = 0;
    int *index;
    for(row = 0; row < rows; row++){
        float *bi = bridgeImage->image.ptr<float>(row);
        char *mi = mask.ptr<char>(row);
        for(col = 0; col < cols; col++){
            if(mi[col] != 0){
                zt = bi[col*ch];
                xt=-(((float)row-(float)rows/2)*zt/f-camHeight);
                if(convertToGrid(xt, zt, mapCol, mapRow) == true){
                    index = &smdml.layer[mi[col]].index[mapRow*mapCols+mapCol].data;
                    if(index < 0){
                        *index = count++;
                    }
                    yt = ((float)rows/2-row)*zt/f;
                    smdml.layer[mi[col]].size[*index].data++;
                    smdml.layer[mi[col]].pt[*index].x += xt;
                    smdml.layer[mi[col]].pt[*index].y += yt;
                    smdml.layer[mi[col]].pt[*index].z += zt;
                }
            }
        }
    }
}

bool darknetImg::convertToGrid(const float& x,const float& y,int& xg,int& yg){
	//マップ上の中心座標(ふつうは　センサ位置＝マップ中心座標　のため　cx=cy=0)
	float cx=0;
	float cy=0;
    //マップ原点座標を画像原点座標(左上)に移動
	float map_x = mapWidth/2 + (x - cx);
	float map_y = mapHeight/2 + ( -(y - cy) );
    //マップ外のデータの際はreturn false
	if(map_x<0 || map_x>mapWidth)
		return false;
	if(map_y<0 || map_y>mapHeight)
		return false;
    //ピクセルデータに変換
	xg = (int)(map_x/mapResolution);
	yg = (int)(map_y/mapResolution);
    //変換成功
	return true;
}

void darknetImg::classifyPoints(){
    
}

void darknetImg::estimatePersonPosition(){

}

void darknetImg::predictPersonPosition(){
    
}

