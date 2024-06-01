#ifndef STEGER_H
#define STEGER_H
#pragma once

#include <QDebug>
#include <iostream>
#include <opencv2/opencv.hpp>

// using namespace cv;
// using namespace std;

class Steger {
public:
    Steger();
    void getCenterLine(int num, cv::Mat& correct_image, cv::Mat& dst_image,
                       std::vector<cv::Point2f>& Points);

private:
    bool stegerMethod(const cv::Mat& src, std::vector<cv::Point2f>& centers);
};

#endif  // STEGER_H
