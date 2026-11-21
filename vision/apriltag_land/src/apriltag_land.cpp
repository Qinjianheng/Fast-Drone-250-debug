#include <ros/ros.h>
#include "apriltag_detector.hpp"

using namespace cv;
using namespace std;

int main()
{
    VideoCapture cap(0);
    AprilTagDetector detector("tagCustom48h12");

    while(ros::ok()){
        Mat img;
        cap >> img;
        if(img.empty()){
            ROS_ERROR("empty img!");
            break;
        }

        detector.detect(img);
        imshow("video", img);
        waitKey(0);
    }

    return 0;
}