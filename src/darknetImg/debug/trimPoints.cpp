#include <obstacle_detection_2019/darknetImgDebug.h>


void darknetImgDebug::addBBGroupRecursively(darknet_ros_msgs::BoundingBoxes& bbs, std::vector<bool>& checkFlag, int coreNumber, int groupNumber){
    int searchNumber = 0;
    std::stringstream ss;
    int text_row = 0;
    checkFlag[coreNumber] = true;
    for(searchNumber = 0; searchNumber < checkFlag.size(); ++searchNumber){
        if(!checkFlag[searchNumber]){
            switch (checkBoundingBoxesRelationship(bbs, coreNumber, searchNumber)) { //他のBBとの関連を調べる
                case darknetImg::Relationship::MIX: //一部含まれている場合は同groupであり、探査BBをcoreにして再度関連を調べる -> 探査済みにする
                    addBBGroupRecursively(bbs, checkFlag, searchNumber, groupNumber);
                    ss << "MIX " << coreNumber << "(" << bbs.bounding_boxes[coreNumber].xmin << "," << bbs.bounding_boxes[coreNumber].ymin << ")-" 
                                    << "(" << bbs.bounding_boxes[coreNumber].xmax << "," << bbs.bounding_boxes[coreNumber].ymax << ")" 
                                    << " : " << searchNumber << "(" << bbs.bounding_boxes[searchNumber].xmin << "," << bbs.bounding_boxes[searchNumber].ymin << ")-"
                                    << "(" << bbs.bounding_boxes[searchNumber].xmax << "," << bbs.bounding_boxes[searchNumber].ymax << ") in" 
                                    << groupNumber << " group";
                    break;
                case darknetImg::Relationship::NONE: //含まれていない場合は別groupであるため、何もしない
                    ss << "NONE " << coreNumber << "(" << bbs.bounding_boxes[coreNumber].xmin << "," << bbs.bounding_boxes[coreNumber].ymin << ")-" 
                                    << "(" << bbs.bounding_boxes[coreNumber].xmax << "," << bbs.bounding_boxes[coreNumber].ymax << ")" 
                                    << " : " << searchNumber << "(" << bbs.bounding_boxes[searchNumber].xmin << "," << bbs.bounding_boxes[searchNumber].ymin << ")-"
                                    << "(" << bbs.bounding_boxes[searchNumber].xmax << "," << bbs.bounding_boxes[searchNumber].ymax << ") in" 
                                    << groupNumber << " group";
                    break;
            }
            if(bridgeImage->encoding == "rgb8"){
                cv::putText(bridgeImage->image, ss.str(), cv::Point(20, 45 + text_row * 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, getColorFromColorMap(5), 2, CV_AA);
                ++text_row;
            }
        }
    }
    ss.str("");
    ss.clear(std::stringstream::goodbit);
    // ROS_INFO_STREAM("BRIDGE : " << bridgeImage->encoding << "TYPE : " << sensor_msgs::image_encodings::TYPE_8UC3);
    if(bridgeImage->encoding == "rgb8"){
        cv::rectangle(bridgeImage->image, cv::Point(bbs.bounding_boxes[coreNumber].xmin, bbs.bounding_boxes[coreNumber].ymin), 
                        cv::Point(bbs.bounding_boxes[coreNumber].xmax, bbs.bounding_boxes[coreNumber].ymax), getColorFromColorMap(groupNumber-1), 5, 8);
        ss << "N" << coreNumber << " : G" << groupNumber;
        cv::putText(bridgeImage->image, ss.str(), cv::Point(bbs.bounding_boxes[coreNumber].xmin, bbs.bounding_boxes[coreNumber].ymin), cv::FONT_HERSHEY_SIMPLEX,
        0.7, getColorFromColorMap(groupNumber-1), 2, CV_AA);
    }
    drawMask(bbs, coreNumber, (char)groupNumber, mask);
}

void darknetImgDebug::publishTrimMask(){
    cv_bridge::CvImage cvi;
    cvi.image = cv::Mat(mask.rows, mask.cols, CV_8UC3, cv::Scalar(50,50,50));
    cvi.encoding = sensor_msgs::image_encodings::RGB8;
    // cvi.encoding = sensor_msgs::image_encodings::MONO8;
    int ch = cvi.image.channels();
    ROS_INFO_STREAM("opencv cvi channnel " << ch);
    for(int row = 0; row < mask.rows; row++){
        auto mask_p = mask.ptr<char>(row);
        auto image_p = cvi.image.ptr<char>(row);
        for(int col = 0; col < mask.cols; col++){
            if(mask_p[col] > 0){
                int colorIndex = ((mask_p[col] - 1)*3)%colorMap.size();
                image_p[col*ch] = colorMap[colorIndex];
                image_p[col*ch+1] = colorMap[colorIndex+1];
                image_p[col*ch+2] = colorMap[colorIndex+2];
            }
        }
    }
    //obstacle_maskを出力する
    // for(int row = 0; row < mask.rows; row++){
    //     auto mask_p = obstacle_mask.ptr<char>(row);
    //     for(int col = 0; col < mask.cols; col++){
    //         if(mask_p[col] > 0){
    //             mask_p[col] = 50;
    //         }
    //     }
    // }
    // cvi.image = obstacle_mask.clone();

    //最終的にパブリッシュ
    trimPoints_pub.publish(cvi.toImageMsg());
}