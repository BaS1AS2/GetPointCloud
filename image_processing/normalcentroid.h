#ifndef NORMALCENTROID_H
#define NORMALCENTROID_H
#pragma once

#include <QDebug>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "image_processing/image_processing.h"

class NormalCentroid {
public:
    NormalCentroid();
    void getCenterLine(int num, cv::Mat& inCorrectImage, cv::Mat& outDstImage,
                       std::vector<cv::Point2f>& Points);

private:
    float ijpixel(float x, float y, cv::Mat& img);
};

#endif  // NORMALCENTROID_H
