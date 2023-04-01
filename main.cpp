#include <opencv2/opencv.hpp>
#include "big_dick_energy.h"

int main() {
    cv::VideoCapture cap;
    cv::Mat frame;
    BigDickEnergy BDE;
//    cv::Mat x = cv::imread("../xxx.png");
//    BDE.pretreatment(x);

    cap.open("../testvideo/output2.mp4");
    cap.set(cv::CAP_PROP_EXPOSURE, + 40);
    while (true) {
        BDE.st = clock();
        cap.read(frame);
        BDE.run(frame);
    }
    return 0;
}
