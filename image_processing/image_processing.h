#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H
#pragma once

#endif  // IMAGE_PROCESSING_H

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <QDebug>
#include <QElapsedTimer>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace ImageProcessing {

extern std::string image_path;  // 用来存放图像文件的路径
extern std::string cloud_path;  // 用来存放点云文件的路径

extern std::vector<float> coeffcient;  // 用来存放从配置文件中解析参数的容器

extern float g_fx;  // 相机在u轴上的尺度因子
extern float g_fy;  // 相机在v轴上的尺度因子
extern float g_u0;  // 相机主点坐标在u轴上的投影位置
extern float g_v0;  // 相机主点坐标在v轴上的投影位置

extern float g_k1;  // 镜头径向畸变系数1
extern float g_k2;  // 镜头径向畸变系数2
extern float g_p1;  // 镜头切向畸变系数1
extern float g_p2;  // 镜头切向畸变系数2

extern float g_sigmaX;          // 高斯滤波的X方向sigma值
extern float g_sigmaY;          // 高斯滤波的Y方向sigma值
extern float g_minThreshold;    // 图像分割最小阈值
extern float g_offset;          // 图像分割每列允许的阈值偏移量
extern float g_region_minArea;  // 面积滤波阈值

extern float g_a;  // 结构光平面方程系数a
extern float g_b;  // 结构光平面方程系数b
extern float g_c;  // 结构光平面方程系数c

extern float g_u;  // 旋转平台旋转轴线方向向量的沿x方向分量
extern float g_v;  // 旋转平台旋转轴线方向向量的沿y方向分量
extern float g_w;  // 旋转平台旋转轴线方向向量的沿z方向分量
extern float g_x;  // 旋转平台旋转轴线上一点的x坐标值
extern float g_y;  // 旋转平台旋转轴线上一点的y坐标值
extern float g_z;  // 旋转平台旋转轴线上一点的z坐标值

int ConfigFileRead(std::string setting_path);  // 解析配置文件

void GetImage(int n, cv::Mat& src_image);  // 取得原图

void Correction(cv::Mat& src_image, cv::Mat& correct_image);  // 畸变校正

void RemoveSmallRegion(cv::Mat& InputImage, cv::Mat& OutputImage, int pixel);  // 去除小的连通域

void GrayCenterHorizontal(cv::Mat& InputImage, std::vector<cv::Point2f>& Pt, cv::Rect bounding_rect,
                          int threshold);  // 灰度重心法（激光条纹水平）

void GrayCenterVertical(cv::Mat& InputImage, std::vector<cv::Point2f>& Pt, cv::Rect bounding_rect,
                        int threshold);  // 灰度重心法（激光条纹垂直）

void CenterLineHorizontal(int m, cv::Mat& correct_image, cv::Mat& dst_image,
                          std::vector<cv::Point2f>& Pt);  // 提取中心线（激光条纹水平）

void CenterLineVertical(cv::Mat& correct_image, cv::Mat& dst_image,
                        std::vector<cv::Point2f>& Pt);  // 提取中心线（激光条纹垂直）

void Point2dto3d(int n, std::vector<cv::Point2f>& Pt,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloud);  // 二维坐标转三维坐标

void Point2dto3dRotaion(int n, std::vector<cv::Point2f>& Pt,
                        std::vector<float> angles);  // 二维坐标转三维坐标（旋转）

void Point2dto3dLinear(int n, std::vector<cv::Point2f>& Pt,
                       std::vector<float> moves,  // 二维坐标转三维坐标（线性）
                       std::vector<float> dir);

}  // namespace ImageProcessing
