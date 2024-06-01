#ifndef CAMERA_CALIBRATION_H
#define CAMERA_CALIBRATION_H
#pragma once

// #include <fstream>
// #include <math.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/sample_consensus/ransac.h>
// #include <pcl/sample_consensus/sac_model_line.h>
// #include <pcl/sample_consensus/sac_model_plane.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <Eigen/Dense>
// #include <Eigen/Eigenvalues>
// #include <ctime>
// #include <iomanip>
// #include <set>

#include <pcl/visualization/pcl_visualizer.h>

#include <QDebug>
#include <QMessageBox>
#include <QTimer>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace Calibration {
/**
 * @brief CamraCalibration  相机标定
 * @param files             文件名
 * @param cameraMatrix      内参矩阵
 * @param distCoeffs        畸变系数
 * @param tvecsMat          平移矩阵
 * @param rvecsMat          旋转矩阵
 */
void CamraCalibration(std::vector<std::string>& files, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
                      std::vector<cv::Mat>& tvecsMat, std::vector<cv::Mat>& rvecsMat);

/**
 * @brief GetImage  取得原图
 * @param img_src   图像名称
 * @param src_image 原始图像
 */
void GetImage(std::string img_src, cv::Mat& src_image);

/**
 * @brief Correction    畸变校正
 * @param src_image     原始图像
 * @param dst_image     结果图像
 * @param cameraMatrix  内参矩阵
 * @param distCoeffs    畸变系数
 */
void Correction(cv::Mat& src_image, cv::Mat& dst_image, cv::Mat& cameraMatrix, cv::Mat& distCoeffs);

/**
 * @brief RemoveSmallRegion 去除面积小于给定值的连通域
 * @param InputImage        输入图像
 * @param OutputImage       输出图像
 * @param pixel             像素值
 */
void RemoveSmallRegion(cv::Mat& InputImage, cv::Mat& OutputImage, int pixel);

/**
 * @brief GrayCenter    灰度重心法提取中心线
 * @param InputImage    输入图像
 * @param Pt            中心点像素坐标
 * @param bounding_rect 连通域外接矩形
 * @param threshold     灰度阈值
 */
void GrayCenter(cv::Mat& InputImage, std::vector<cv::Point2d>& Pt, cv::Rect bounding_rect,
                int threshold);

// 对点的坐标按x从小到大排序，若x值相同则按y从小到大排序//
bool PointSortRule(const cv::Point2d pt1, const cv::Point2d pt2);

// 亚像素级激光条纹中心线提取//
void CenterLine(int n, cv::Mat& correct_image, std::vector<cv::Point2d>& P);

/**
 * @brief GrayCenter_vertical    灰度重心法（激光条纹垂直）
 * @param InputImage				输入图像
 * @param Pt						中心点坐标值
 * @param bounding_rect			连通域轮廓外接矩形
 * @param threshold				灰度阈值
 */
void GrayCenterVertical(cv::Mat& InputImage, std::vector<cv::Point2d>& Pt, cv::Rect bounding_rect,
                        int threshold);

/**
 * @brief GrayCenter_horizontal  灰度重心法（激光条纹水平）
 * @param InputImage				输入图像
 * @param Pt						中心点坐标值
 * @param bounding_rect			连通域轮廓外接矩形
 * @param threshold				灰度阈值
 */
void GrayCenterHorizontal(cv::Mat& InputImage, std::vector<cv::Point2d>& Pt, cv::Rect bounding_rect,
                          int threshold);

/**
 * @brief CenterLine_horizontal  提取中心线（激光条纹水平）
 * @param correct_image			校正后的图像
 * @param dst_image				处理结果图
 * @param Pt						中心点坐标值
 */
void CenterLineHorizontal(int m, cv::Mat& correct_image, cv::Mat& dst_image,
                          std::vector<cv::Point2d>& Pt);

/**
 * @brief GrayCenter_vertical    灰度重心法（激光条纹垂直）
 * @param InputImage				输入图像
 * @param Pt						中心点坐标值
 * @param bounding_rect			连通域轮廓外接矩形
 * @param threshold				灰度阈值
 */
void CenterLineVertical(int i, cv::Mat& correct_image, cv::Mat& dst_image,
                        std::vector<cv::Point2d>& Pt);

/**
 * @brief Point2dSperate    分离坐标点
 * @param P                 中心点坐标值
 * @param P_plane           平面上的中心点
 * @param P_object          物体上的中心点
 */
void Point2dSperate(std::vector<cv::Point2d>& P, std::vector<cv::Point2d>& P_plane,
                    std::vector<cv::Point2d>& P_object);

/**
 * @brief Point2dto3d   二维像素坐标转三维图像坐标
 * @param n             图像序号
 * @param plane         平面方程系数
 * @param cameraMatrix  内参矩阵
 * @param distCoeffs    畸变系数
 * @param Pt2ds         二维点集
 * @param Pt3ds         三维点集
 */
void Point2dto3d(int n, std::vector<double> plane, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
                 std::vector<cv::Point2d>& Pt2ds, std::vector<cv::Point3d>& Pt3ds);

/**
 * @brief PointtoPlaneEvaluation    点到面精度评价
 * @param Pt3ds                     三维点集
 * @param plane                     平面方程系数
 */
void PointtoPlaneEvaluation(const std::vector<cv::Point3d>& Pt3ds, std::vector<double> plane);

/**
 * @brief findPlane 最小二乘法拟合平面
 * @param pts       输入三维点集
 * @return          平面方程系数
 */
std::vector<double> PlaneFittingRansac(std::vector<cv::Point3d>& pts);

}  // namespace Calibration

#endif  // CAMERA_CALIBRATION_H
