#include<obstacle_detection_2019/imageMatching.h>

imageMatchingClass::imageMatchingClass()
{

	//subscriber
	nhSub1.setCallbackQueue(&queue1);
	subImg=nhSub1.subscribe("/zed/left/image_rect_color",1,&imageMatchingClass::image_callback,this);
	nhSub2.setCallbackQueue(&queue2);
	subMskImg=nhSub2.subscribe("maskImageData",1,&imageMatchingClass::maskImage_callback,this);
	//publisher
    pubMatch= nhPub.advertise<obstacle_detection_2019::ImageMatchingData>("imageMatchingData", 1);

    //localMapParameter
    //後でlaunchから読み込みように変更
	mapW=8;//width[m]
	mapH=8;//height[m]
	mapR=0.05;//resolution[m]
	mapWi=(int)(mapW/mapR);//[pixel]
	mapHi=(int)(mapH/mapR);//[pixel]
	
	//特徴点抽出
	//画像分割数
	//分割のアスペクト比が均等に近く, 割り切れる値を選出
	nh=8;
	nw=16;
	//max検出数（分割領域１つあたり）
	maxDetectPoint=20;
	//max取得数（分割領域１つあたり）
	maxPoint=10;
	//window size
	ws=13;
}
imageMatchingClass::~imageMatchingClass(){
	
}