#include "image_processing/normalcentroid.h"

NormalCentroid::NormalCentroid() {}

#include <iostream>
#include <opencv2/opencv.hpp>

// 亚像素灰度值计算
float NormalCentroid::ijpixel(float x, float y, cv::Mat& img) {
    int x_0 = int(x);
    int x_1 = int(x + 1);
    int y_0 = int(y);
    int y_1 = int(y + 1);
    int px_0y_0 = int(img.at<uchar>(y_0, x_0));
    int px_0y_1 = int(img.at<uchar>(y_1, x_0));
    int px_1y_0 = int(img.at<uchar>(y_0, x_1));
    int px_1y_1 = int(img.at<uchar>(y_1, x_1));
    float x_y0 = px_0y_0 + (x - float(x_0)) * (px_1y_0 - px_0y_0);
    float x_y1 = px_0y_1 + (x - float(x_0)) * (px_1y_1 - px_0y_1);
    float x_y = x_y0 + (y - float(y_0)) * (x_y1 - x_y0);
    return x_y;
}

void NormalCentroid::getCenterLine(int num, cv::Mat& inCorrectImage, cv::Mat& outDstImage,
                                   std::vector<cv::Point2f>& Points) {
    cv::Mat src_img = inCorrectImage.clone();
    outDstImage = src_img.clone();
    cv::cvtColor(outDstImage, outDstImage, cv::COLOR_GRAY2RGB);

    uchar* p = src_img.data;
    std::vector<cv::Point2f> pts;

    cv::Mat img1 = src_img.clone();
    cv::Mat img2;

    cv::GaussianBlur(img1, img1, cv::Size(0, 0), 1.2, 1.2);  // 高斯滤波

    // 求每列灰度值最大值
    uchar* p2 = img1.data;
    std::vector<int> max_col_scalar(img1.cols);

    for (int i = 0; i < img1.cols; ++i) {
        for (int j = 0; j < img1.rows; ++j) {
            if (*(p2 + i + img1.cols * j) > max_col_scalar[i]) {
                max_col_scalar[i] = *(p2 + i + img1.cols * j);
            }
        }
    }

    // 按列阈值操作
    int pixels = 0;
    p2 = img1.data;
    for (int i = 0; i < img1.cols; ++i) {
        //        int threshold = std::max(max_col_scalar[i] - (float)30, g_minThreshold);
        int threshold = std::max(max_col_scalar[i] - 20, 100);

        for (int j = 0; j < img1.rows; ++j) {
            if (*(p2 + i + img1.cols * j) < threshold)
                *(p2 + i + img1.cols * j) = 0;
            else
                ++pixels;
        }
    }

    //    if (pixels < 1000) return;
    ImageProcessing::RemoveSmallRegion(img1, img2, 30);  // 面积滤波

    cv::GaussianBlur(img2, img2, cv::Size(11, 11), 1.2, 1.2);  // 高斯滤波

    p = img2.data;
    src_img = img2.clone();

    // 灰度重心法
    for (size_t i = 0; i < img2.cols; ++i) {
        int sum = 0;
        float y = 0;

        for (size_t j = 0; j < img2.rows; ++j) {
            int s = *(p + i + img2.cols * j);
            if (s) {
                sum += s;
                y += j * s;
            }
        }
        if (sum) {
            y /= sum;
            pts.push_back(cv::Point2f(i, y));
        }
    }

    // 法向迭代质心法
    int iter = 0;
    int max_iter = 100;
    float distance = 0;
    float thre_distance = (float)0.01;
    std::vector<cv::Point2f> pts_new = pts;

    do {
        int pts_tmp_size = 5;
        assert((pts_tmp_size - 1) % 2 == 0);

        for (size_t i = (pts_tmp_size - 1) % 2; i < pts.size() - (pts_tmp_size - 1) % 2; ++i) {
            std::vector<cv::Point2f> pts_tmp;
            for (size_t j = 0; j < pts_tmp_size; ++j) {
                pts_tmp.push_back(pts[i + j - (pts_tmp_size - 1) % 2]);
            }

            cv::Vec4f line_para;
            cv::fitLine(pts_tmp, line_para, cv::DIST_L2, 0, 1e-2,
                        1e-2);  // 最小二乘拟合直线方程系数
            float k = line_para[1] / line_para[0];
            float sin_theta = 1 / sqrt(k * k + 1);
            float cos_theta = -k / sqrt(k * k + 1);

            float sum = ijpixel(pts[i].x, pts[i].y, src_img);
            float sumx = ijpixel(pts[i].x, pts[i].y, src_img) * pts[i].x;
            float sumy = ijpixel(pts[i].x, pts[i].y, src_img) * pts[i].y;

            int range = 10;
            for (size_t j = 1; j < range; ++j) {
                float x_cor_left = pts[i].x - j * cos_theta;
                float y_cor_left = pts[i].y - j * sin_theta;
                float x_cor_right = pts[i].x + j * cos_theta;
                float y_cor_right = pts[i].y + j * sin_theta;

                if (x_cor_left >= 0 && x_cor_left < src_img.cols && y_cor_left >= 0 &&
                    y_cor_left < src_img.rows && x_cor_right >= 0 && x_cor_right < src_img.cols &&
                    y_cor_right >= 0 && y_cor_right < src_img.rows) {
                    if (ijpixel(x_cor_left, y_cor_left, src_img) ||
                        ijpixel(x_cor_right, y_cor_right, src_img)) {
                        sum += ijpixel(x_cor_left, y_cor_left, src_img) +
                               ijpixel(x_cor_right, y_cor_right, src_img);
                        sumx += ijpixel(x_cor_left, y_cor_left, src_img) * x_cor_left +
                                ijpixel(x_cor_right, y_cor_right, src_img) * x_cor_right;
                        sumy += ijpixel(x_cor_left, y_cor_left, src_img) * y_cor_left +
                                ijpixel(x_cor_right, y_cor_right, src_img) * y_cor_right;
                    }
                }
            }

            pts_new[i].x = (float)sumx / sum;
            pts_new[i].y = (float)sumy / sum;
        }

        distance = 0;
        for (int i = 0; i < pts.size(); i++) {
            distance += pow(pts_new[i].x - pts[i].x, 2) + pow(pts_new[i].y - pts[i].y, 2);
        }
        distance = sqrt(distance / pts.size());
        iter++;
        qDebug() << iter << "\t" << distance;

        pts = pts_new;

        cv::Mat tempImg = outDstImage.clone();
        for (size_t i = 0; i < pts.size(); ++i) {
            cv::circle(tempImg, cv::Point(round(pts[i].x), round(pts[i].y)), 0,
                       cv::Scalar(0, 0, 255), -1);
        }
        cv::imwrite(
            "./data/center/normal" + std::to_string(num) + "." + std::to_string(iter) + ".bmp",
            tempImg);

    } while (iter < max_iter && distance > thre_distance);

    Points.clear();
    for (size_t i = 0; i < pts.size(); ++i) {
        Points.push_back(cv::Point2f(pts[i].x, pts[i].y));

        cv::circle(outDstImage, cv::Point(round(pts[i].x), round(pts[i].y)), 0,
                   cv::Scalar(0, 0, 255), -1);
    }

    cv::imwrite("./data/center/normal" + std::to_string(num) + ".0.bmp", outDstImage);

    //    system("pause");
    //    return EXIT_SUCCESS;
}
