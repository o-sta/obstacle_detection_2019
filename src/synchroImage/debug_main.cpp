#include <obstacle_detection_2019/synchroImage.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"obstacle_detection_2019_syncro");
	
    syncroImageClass sic; //syncro class

    ros::spin();

	return 0;
}