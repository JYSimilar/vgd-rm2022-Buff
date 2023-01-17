#include <opencv2/opencv.hpp>
#include "big_dick_energy.h"

int main() {
    cv::VideoCapture cap;
    cv::Mat frame;
    BigDickEnergy BDE;
    cv::Mat x = cv::imread("../xxx.png");
    BDE.pretreatment(x);

//    cap.open("/Users/jysimilar/Downloads/rm/能量机关/2022中部赛区-大符-西南大学/西南大学GKD-大符内录/output2.mp4");
//    cap.set(cv::CAP_PROP_EXPOSURE, + 40);
//    while (true) {
//        BDE.st = clock();
//        cap.read(frame);
//        BDE.run(frame);
//    }
    return 0;
}
