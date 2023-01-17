//
// Created by JYSimilar on 2022/12/5.
// Latest change by JYSimilar on 2022/12/14
//

#ifndef RM2023_BIGDICKENERGY_BIG_DICK_ENERGY_H
#define RM2023_BIGDICKENERGY_BIG_DICK_ENERGY_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include "define.h"
#include "predictor.h"


class BigDickEnergy {
private:
    double sum = 0;
    int times = 0;
public:
    Predictor predictor;
    std::clock_t st = 0, pre = 0, vtx = 0;
    long double preW = 0, angleDif = 0;
    long double tmpW;
    cv::Mat functionSin;
    // 以上均为计时及计算角速度等用

private:
    struct BuffPara {
        int grayThreshold_RED = 70;                            //灰度二值化阈值-红色
        int grayThreshold_BLUE = 60;                            //灰度二值化阈值-蓝色
        int separationThreshold_RED = 70;                    //色彩分离二值化阈值-红色
        int separationThreshold_BLUE = 70;                    //色彩分离二值化阈值-蓝色
        float imageBright_RED = -10;                         //亮度削减-红色
        float imageBright_BLUE = -10;                         //亮度削减-蓝色
        float imageBright_GRAY = -10;                         //亮度削减-灰度二值化
    };
    BuffPara _para;
    cv::Mat bin;
    cv::Mat m_frame;
    cv::Mat templ[9];
    cv::Mat m_element1 = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::Mat m_element2 = getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::Mat m_element3 = getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    cv::Mat m_element4 = getStructuringElement(cv::MORPH_RECT, cv::Size(21, 21));

    cv::Rect2f ROI;                                           // 扇叶ROI

    // debug
    cv::Mat sw;

    cv::Point2d centerR;                                      // R标位置（圆心）
    cv::Point2d centerP;                                        // 击打点

public:
    bool m_isTracking = false;                              // 是否处于跟踪状态

private:
    const int m_LittleAngularVelocity = 10;                 // 小能量机关转速
    const double PI = 3.1415926535897932384626;

    cv::Point2f Pnt[4];                                     // 实际矩形四点
    cv::Point2d prediction;                                   // 预测点
    double radius = 0;                                          // 路径半径
    long double angle = 0, pre_angle = 0;                        // 当前扇叶角度
    long double w = 0;                                           // 两帧之间的角速度

public:
    BigDickEnergy();
    void run(cv::Mat frame);

public:
    void pretreatment(cv::Mat frame);
    void findContours();
    bool Tracker();
    double getDistance(cv::Point2f A, cv::Point2f B);
    double TemplateMatch(cv::Mat image, cv::Mat tepl, cv::Point &point, int method);
    void debug();

};


#endif //RM2023_BIGDICKENERGY_BIG_DICK_ENERGY_H
