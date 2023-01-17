//
// Created by JYSimilar on 2023/1/6.
//

#include "predictor.h"


double Predictor::changeIntoLine(double angle) {
    bool isR = angle >= 0;
    if (t^isR) {
        ++rounds;
    }
    t = isR;
    double t_angle = fabs(angle);
    cv::Point2f pos(t_angle * 500.0, 30.0 + 20.0 * rounds);
    cv::circle(functionSin, pos, 1, cv::Scalar (0, 255, 0));
    cv::imshow("sin2", functionSin);
}