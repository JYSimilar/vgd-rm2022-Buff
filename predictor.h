//
// Created by JYSimilar on 2023/1/6.
//

#ifndef RM2023_BIGDICKENERGY_PREDICTOR_H
#define RM2023_BIGDICKENERGY_PREDICTOR_H


#include <opencv2/opencv.hpp>
// #include <eigen3/Eigen/Core>
// #include <eigen3/Eigen/Dense>
#include <cmath>


class Predictor {
private:
    int rounds = 0;
    bool tmp = false;
    cv::Mat functionSin = cv::Mat(600, 1800, CV_8UC3,cv::Scalar(0,0,0));
    const double PI = 3.1415926535897932384626;
public:
    std::vector<double> gaussNewton(std::vector<clock_t> t, std::vector<double> spd);
};


#endif //RM2023_BIGDICKENERGY_PREDICTOR_H
