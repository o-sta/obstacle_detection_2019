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
            cv::putText(bridgeImage->image, ss.str(), cv::Point(20, 45 + text_row * 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, getColorFromColorMap(5), 2, CV_AA);
            ++text_row;
        }
    }
    ss.str("");
    ss.clear(std::stringstream::goodbit);
    cv::rectangle(bridgeImage->image, cv::Point(bbs.bounding_boxes[coreNumber].xmin, bbs.bounding_boxes[coreNumber].ymin), 
                  cv::Point(bbs.bounding_boxes[coreNumber].xmax, bbs.bounding_boxes[coreNumber].ymax), getColorFromColorMap(groupNumber-1), 5, 8);
    ss << "N" << coreNumber << " : G" << groupNumber;
    cv::putText(bridgeImage->image, ss.str(), cv::Point(bbs.bounding_boxes[coreNumber].xmin, bbs.bounding_boxes[coreNumber].ymin), cv::FONT_HERSHEY_SIMPLEX,
    0.7, getColorFromColorMap(groupNumber-1), 2, CV_AA);
    drawMask(bbs, coreNumber, (char)groupNumber, mask);
}