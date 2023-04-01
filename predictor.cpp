//
// Created by JYSimilar on 2023/1/6.
//

#include "predictor.h"
#include <Eigen/Core>
#include <Eigen/Dense>

std::vector<double> Predictor ::gaussNewton(std::vector<clock_t> t, std::vector<double> spd) {
    double ae = 0.780, we = 1.884;          // 初始值
    double t_0 = 0;

    const int N = t.size();                 // 数据量
    double w_sigma = 0.0017;                // 噪声
    double inv_sigma = 1.0 / w_sigma;

    const int iterations = 50;                   // 迭代次数10
    for (int i = 0; i < iterations; ++i) {
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        Eigen::Vector3d b = Eigen::Vector3d::Zero();

        for (int j = 0; j < N; ++j) {
            clock_t xi = t[j];
            double yi = spd[j];
            double error = yi - (ae * sin(we * (xi - t_0))) - 2.09 + ae;
            Eigen::Vector3d J;
            J[0] = - sin(we * (xi - t_0)) + 1;
            J[1] = - ae * we * cos(we * (xi - t_0));
            J[2] = ae * we * cos(we * (xi - t_0));
            H += inv_sigma * inv_sigma * J * J.transpose();
            b += -inv_sigma * inv_sigma * error * J;
        }

        Eigen::Vector3d dx = H.ldlt().solve(b);
        if (isnan(dx[0])) {
            return {};
        }
        ae += dx[0];
        we += dx[1];
        t_0 += dx[2];
    }
    std::cout<<"ae = " << ae << ", we = " << we << ", t0 = " << t_0 << std::endl;
    return {ae, we, t_0};
}