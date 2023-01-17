//
// Created by JYSimilar on 2022/12/5.
// Latest change by JYSimilar on 2022/12/9
//

#include "big_dick_energy.h"


// 构造函数中初始化模板
BigDickEnergy::BigDickEnergy() {
    // 查看大能量机关函数图像用的
    functionSin = cv::Mat(600, 1800, CV_8UC3,cv::Scalar(0,0,0));
    // 提前生成需要的模板

    // 防止不小心用到这个空指针，让其赋值，变为非空指针
    templ[0] = cv::imread("../template/" + std::to_string(1) + ".jpg", cv::COLOR_BGR2BGRA);
    cv::resize(templ[0], templ[0], cv::Size(42,20));

    for(int i = 1; i <= 8; i++) {
        // 读入模板
        templ[i] = cv::imread("../template/" + std::to_string(i) + ".jpg", cv::COLOR_BGR2BGRA);
        // 修改模板大小
        cv::resize(templ[i], templ[i], cv::Size(42,20));
    }
}



void BigDickEnergy::run(cv::Mat frame) {
    // 预处理
    pretreatment(frame);

    // 追踪，如果处于追踪态一定有追踪值
    Tracker();

    // 识别，
    // 如果追踪到且找到，会使用真实值
    // 如果追踪到且没找到，会使用追踪值
    // 如果两者都没，则没有找到装甲板
    // 永远会优先重置追踪ROI到真实值位置，避免一直追踪已击打装甲板
    findContours();

    // debug用，显示图像
    // 追踪的ROI图像显示不在这里，在Tracker函数中
    debug();
}


// 预处理图像
void BigDickEnergy::pretreatment(cv::Mat frame) {
    frame.copyTo(m_frame);

    cv::Mat gray;
    std::vector<cv::Mat> bgr;
    cvtColor(m_frame, gray, cv::COLOR_BGR2GRAY);
    split(m_frame, bgr);
    cv::Mat gray_bin, color_bin;

#ifdef DETECT_BLUE
    cv::threshold(gray, gray_bin, _para.grayThreshold_BLUE, 255, cv::THRESH_BINARY);
    cv::subtract(bgr[0], bgr[2], color_bin);
    cv::threshold(color_bin, color_bin, _para.separationThreshold_BLUE, 255, cv::THRESH_BINARY);
#endif
#ifdef DETECT_RED
    cv::threshold(gray, gray_bin, _para.grayThreshold_RED, 255, cv::THRESH_BINARY);
    cv::subtract(bgr[2], bgr[0], color_bin);
    cv::threshold(color_bin, color_bin, _para.separationThreshold_RED, 255, cv::THRESH_BINARY);
#endif

    bin = gray_bin & color_bin;
    // cv::morphologyEx(bin, bin, cv::MORPH_CLOSE, m_element3);


    cv::imshow("t",bin);
    cv::waitKey(0);

}


// 寻找装甲板位置及轮廓并计算路径等
void BigDickEnergy::findContours() {
    // 找轮廓
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(bin, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

    cv::RotatedRect rect_tmp2;
    bool findTarget = false;

    if(!hierarchy.empty()) {
        for(int i = 0; i >= 0; i = hierarchy[i][0]) {
            // 找出轮廓最小正接矩形
            rect_tmp2 = minAreaRect(contours[i]);

            // 存矩形四点
            cv::Point2f P[4];
            rect_tmp2.points(P);

            cv::Point2f srcRect[4];
            cv::Point2f dstRect[4];

            double width;
            double height;

            //矫正提取的叶片的宽高
            width = getDistance(P[0], P[1]);
            height = getDistance(P[1], P[2]);

            if (width > height) {
                srcRect[0] = P[0];
                srcRect[1] = P[1];
                srcRect[2] = P[2];
                srcRect[3] = P[3];
            } else {
                std::swap(width, height);
                srcRect[0] = P[1];
                srcRect[1] = P[2];
                srcRect[2] = P[3];
                srcRect[3] = P[0];
            }

            double area = height * width;
            if(area > 3000) {
                dstRect[0] = cv::Point2f(0, 0);
                dstRect[1] = cv::Point2f(width, 0);
                dstRect[2] = cv::Point2f(width, height);
                dstRect[3] = cv::Point2f(0, height);

                // 应用透视变换，矫正成规则矩形
                cv::Mat transform = getPerspectiveTransform(srcRect, dstRect);
                cv::Mat perspectiveMat;
                warpPerspective(bin, perspectiveMat, transform, bin.size());

                // 提取扇叶图片
                cv::Mat testImg;
                testImg = perspectiveMat(cv::Rect(0, 0, width, height));

                // 模板匹配
                cv::Point matchLoc;
                double value;
                cv::Mat tmp1;

                // 统一大小，模板匹配中被匹配图像需要大于等于模板大小
                // 当模板和被匹配物体大小相近时匹配效果最好
                resize(testImg, tmp1, cv::Size(42,20));
                // 用于保存匹配得分
                std::vector<double> Vvalue1;        // 识别待打击的扇叶
                std::vector<double> Vvalue2;        // 识别已经打击过的扇叶
                for(int j = 1; j <= 6; j++) {
                    value = TemplateMatch(tmp1, templ[j], matchLoc, cv::TM_CCOEFF_NORMED);
                    Vvalue1.push_back(value);
                }
                for(int j = 7; j <= 8; j++) {
                    value = TemplateMatch(tmp1, templ[j], matchLoc, cv::TM_CCOEFF_NORMED);
                    Vvalue2.push_back(value);
                }

                int maxv1 = 0, maxv2 = 0;
                // 找出匹配值最大的序号
                for(int t1 = 0; t1 < 6; t1++) {
                    if(Vvalue1[t1] > Vvalue1[maxv1]) {
                        maxv1 = t1;
                    }
                }
                for(int t2 = 0; t2 < 2; t2++) {
                    if(Vvalue2[t2] > Vvalue2[maxv2]) {
                        maxv2 = t2;
                    }
                }

                // 匹配条件
                if(Vvalue1[maxv1] > Vvalue2[maxv2] && Vvalue1[maxv1] > 0.4) {
                    findTarget = true;
                    if(hierarchy[i][2] > 0) {
                        cv::imshow("t", testImg);
                        cv::RotatedRect rect_tmp = minAreaRect(contours[hierarchy[i][2]]);
                        rect_tmp.points(Pnt);

                        // 用于筛选装甲板
//                        const float maxHWRatio = 0.7153846;
//                        const float maxArea = 2000;
//                        const float minArea = 500;

                        float width = rect_tmp.size.width;
                        float height = rect_tmp.size.height;
                        if (height > width) {
                            std::swap(height, width);
                        }
                        centerP = rect_tmp.center;

                        // 更新ROI
                        for (int j = 0; j < 4; ++j) {
                            if (j == 0) ROI.width = getDistance(Pnt[j], Pnt[(j + 1) % 4]);
                            else if (j == 1) ROI.height = getDistance(Pnt[j], Pnt[(j + 1) % 4]);
                        }
                        if (ROI.width < ROI.height) {
                            std::swap(ROI.width, ROI.height);
                        }
                        ROI.width *= 2.5;
                        ROI.height *= 3.6;
                        ROI.x = centerP.x - ROI.width/2;
                        ROI.y = centerP.y - ROI.height/2;
                        if (ROI.x < 0) {
                            ROI.width += ROI.x;
                            ROI.x = 0;
                        }
                        if (ROI.y < 0) {
                            ROI.height += ROI.y;
                            ROI.y = 0;
                        }

                        m_isTracking = true;
                    } else {
                        continue;
                    }
                } else {
                    continue;
                }
            } else if (area > 30 && area < 500) {
                // 找 R标
                cv::Point2f RRectP[4];
                rect_tmp2.points(RRectP);
                // -1.3 , +5.7
                centerR.x = (RRectP[0].x - RRectP[2].x)/2 + RRectP[2].x + 0;
                centerR.y = (RRectP[0].y - RRectP[2].y)/2 + RRectP[2].y - 0.3;

            } else {
                continue;
            }
        }

        // 计算半径
        double XDifference = centerP.x - centerR.x;
        double YDifference = centerP.y - centerR.y;
        radius = sqrt(XDifference * XDifference + YDifference * YDifference);
        // 算出角度
        pre_angle = angle;
        angle = atan2(YDifference, XDifference);

        if (!true) {         // 小能量机关
            // 预测0.5秒后的点
            // 预测角度
            double nxtAngle = angle + PI / 12;
            // 预测坐标
            prediction.y = (radius * sin(nxtAngle)) + centerR.y;
            prediction.x = (radius * cos(nxtAngle)) + centerR.x;

        } else {            // 大能量机关
            // 计算角速度
            pre = vtx;
            vtx = clock();
            preW = tmpW;
            angleDif = (fabs(angle - pre_angle)/2*PI);
            tmpW = angleDif / ((double)(vtx - pre) * 0.001);
            if (fabs(tmpW - w) < 0.0023 || fabs(tmpW - preW) < 0.0023) {
                w = tmpW;
                std::cout << w << std::endl;
            }
            predictor.changeIntoLine(angle);
        }
    }
}


// 补帧ROI
bool BigDickEnergy::Tracker() {
    cv::Mat frame;
    m_frame.copyTo(frame);

    if (ROI.empty()) {
        return false;
    }

    // 单独切出来的ROI
    cv::Mat testImg;
    testImg = bin(cv::Rect(ROI.x, ROI.y, ROI.width, ROI.height));

    // 找轮廓
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(testImg, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
    cv::RotatedRect rect_tmp2;

    // 直接找，不用模板匹配，因为划定了ROI
    if (!hierarchy.empty()) {
        for(int i = 0; i >= 0; i = hierarchy[i][0]) {
            // 找出轮廓最小正接矩形
            rect_tmp2 = minAreaRect(contours[i]);
            // 存矩形四点
            cv::Point2f P[4];
            rect_tmp2.points(P);

            if(hierarchy[i][2] > 0) {
                cv::RotatedRect rect_tmp = minAreaRect(contours[hierarchy[i][2]]);
                rect_tmp.points(Pnt);
                float width = rect_tmp.size.width;
                float height = rect_tmp.size.height;
                if (height > width) {
                    std::swap(height, width);
                }

                centerP = rect_tmp.center;

                // 修正追踪的击打点及装甲板轮廓
                centerP.x += ROI.x;
                centerP.y += ROI.y;
                for (auto & j : Pnt) {
                    j.x += ROI.x;
                    j.y += ROI.y;
                }

                // 更新ROI
                for (int j = 0; j < 4; ++j) {
                    if (j == 0) ROI.width = getDistance(Pnt[j], Pnt[(j + 1) % 4]);
                    else if (j == 1) ROI.height = getDistance(Pnt[j], Pnt[(j + 1) % 4]);
                }
                if (ROI.width < ROI.height) {
                    std::swap(ROI.width, ROI.height);
                }
                ROI.width *= 2.5;
                ROI.height *= 3.6;
                ROI.x = centerP.x - ROI.width/2;
                ROI.y = centerP.y - ROI.height/2;
                if (ROI.x < 0) {
                    ROI.width += ROI.x;
                    ROI.x = 0;
                }
                if (ROI.y < 0) {
                    ROI.height += ROI.y;
                    ROI.y = 0;
                }

            }
        }
    } else {
        m_isTracking = false;
        return false;
    }

    // debug
    // 打击点
    cv::circle(frame, centerP, 5, cv::Scalar(0, 0, 255), 2);
    // 画出装甲位置，并更新ROI
    for (int j = 0; j < 4; ++j) {
        cv::line(frame, Pnt[j], Pnt[(j + 1) % 4], cv::Scalar(0, 255, 255), 3);
    }
    // 画出ROI在原图中范围
    cv::rectangle(frame, ROI, cv::Scalar(0, 100, 100), 2);
    // 显示图像
    cv::imshow("ROI", frame);
    // 显示ROI内图像
    cv::imshow("ROI_inside", testImg);

    return true;
}


// debug
void BigDickEnergy::debug() {

    cv::imshow("pretreatment", bin);

    m_frame.copyTo(sw);

    // 画圆形路径
    if (radius < 500) cv::circle(sw, centerR, radius, cv::Scalar(255, 0, 255), 1);

    // 画中心R标
    cv::circle(sw, centerR, 20, cv::Scalar(0, 255, 0), 2);

    // 画预测点
    cv::circle(sw, prediction, 9, cv::Scalar(0, 255, 100), 3);

    // 画实际矩形
    for (int j = 0; j < 4; ++j) {
        cv::line(sw, Pnt[j], Pnt[(j + 1) % 4], cv::Scalar(0, 255, 255), 3);
    }

    // 画实际打击点
    cv::circle(sw, centerP, 5, cv::Scalar(0, 0, 255), 2);

    // 画大能量机关函数图
    cv::Point2f pos((float)(st/10000 % 1800), 500 - (float)w*20000);
    cv::circle(functionSin, pos, 1, cv::Scalar (0, 255, 0));
    cv::imshow("sin", functionSin);

    // 显示图像
    cv::imshow("impact", sw);

    cv::waitKey(1);
}


// 计算两点距离
double BigDickEnergy::getDistance(cv::Point2f A, cv::Point2f B) {
    double dis;
    dis = pow((A.x - B.x), 2) + pow((A.y - B.y), 2);
    return sqrt(dis);
}


// 模板匹配，网上抄来的
double BigDickEnergy::TemplateMatch(cv::Mat image, cv::Mat tepl, cv::Point &point, int method){
    int result_cols =  abs(image.cols - tepl.cols + 1);
    int result_rows = abs(image.rows - tepl.rows + 1);

    cv::Mat result = cv::Mat(result_cols, result_rows, CV_32FC1);
    cv::matchTemplate(image, tepl, result, method);

    double minVal, maxVal;
    cv::Point minLoc, maxLoc;
    cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );

    switch(method)
    {
        case cv::TM_SQDIFF:
        case cv::TM_SQDIFF_NORMED:
            point = minLoc;
            return minVal;

        default:
            point = maxLoc;
            return maxVal;

    }
}

