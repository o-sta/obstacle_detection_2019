#include <obstacle_detection_2019/management.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"obstacle_detection_2019_manage");
	
    managementClass mc; //syncro class
    while(ros::ok()){
        mc.subscribeData();
    }

	return 0;
}