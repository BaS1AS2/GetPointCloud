#include "image_processing/image_processing.h"

namespace ImageProcessing {

/**
 * @brief ConfigFileRead  解析配置文件
 * @param setting_path	  配置文件路径
 * @return				  状态码
 */
int ConfigFileRead(std::string setting_path) {
    std::ifstream configFile;
    configFile.open(setting_path.c_str());
    std::string strLine;
    coeffcient.clear();

    if (configFile.is_open()) {
        std::cout << "Open config file!" << std::endl;
        while (!configFile.eof()) {
            getline(configFile, strLine);
            // 解析图像储存路径
            size_t pos_image_path = strLine.find("image path");
            if (pos_image_path != std::string::npos) {
                image_path = strLine.substr(pos_image_path + 11);
            }
            // 解析点云储存路径
            size_t pos_cloud_path = strLine.find("cloud path");
            if (pos_cloud_path != std::string::npos) {
                cloud_path = strLine.substr(pos_cloud_path + 11);
            }
            // 解析其他参数设置
            size_t pos_val = strLine.find('=');
            if (pos_image_path == std::string::npos && pos_cloud_path == std::string::npos &&
                pos_val != std::string::npos) {
                std::string val = strLine.substr(pos_val + 1);
                float coeff = atof(val.c_str());
                if (coeff == 0) return 0;
                coeffcient.push_back(coeff);
            }
        }

        if (coeffcient.size() != 22) return 0;

        g_fx = coeffcient[0];
        g_u0 = coeffcient[1];
        g_fy = coeffcient[2];
        g_v0 = coeffcient[3];
        g_k1 = coeffcient[4];
        g_k2 = coeffcient[5];
        g_p1 = coeffcient[6];
        g_p2 = coeffcient[7];
        g_sigmaX = coeffcient[8];
        g_sigmaY = coeffcient[9];
        g_minThreshold = coeffcient[10];
        g_offset = coeffcient[11];
        g_region_minArea = coeffcient[12];
        g_a = coeffcient[13];
        g_b = coeffcient[14];
        g_c = coeffcient[15];
        g_u = coeffcient[16];
        g_v = coeffcient[17];
        g_w = coeffcient[18];
        g_x = coeffcient[19];
        g_y = coeffcient[20];
        g_z = coeffcient[21];
    } else {
        std::cout << "Cannot open config file!" << std::endl;
        return -1;
    }
    return 1;
}

/**
 * @brief GetImage    取得原图
 * @param n           图像序号
 * @param src_image   原图
 */
void GetImage(int n, cv::Mat& src_image) {
    src_image = cv::imread("./data/image/" + std::to_string(n) + ".bmp", cv::IMREAD_GRAYSCALE);
    //    src_image = cv::imread("./data/image/1.bmp", cv::IMREAD_GRAYSCALE);
}

/**
 * @brief Correction      畸变校正
 * @param src_image       原图
 * @param correct_image   校正结果
 */
void Correction(cv::Mat& src_image, cv::Mat& correct_image) {
    cv::Mat cameraMatrix = cv::Mat::zeros(3, 3, CV_32F);  // 相机内参矩阵
    cameraMatrix.at<float>(0, 0) = g_fx;
    cameraMatrix.at<float>(0, 2) = g_u0;
    cameraMatrix.at<float>(1, 1) = g_fy;
    cameraMatrix.at<float>(1, 2) = g_v0;
    cameraMatrix.at<float>(2, 2) = 1;

    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_32F);  // 畸变系数
    distCoeffs.at<float>(0, 0) = g_k1;
    distCoeffs.at<float>(1, 0) = g_k2;
    distCoeffs.at<float>(2, 0) = g_p1;
    distCoeffs.at<float>(3, 0) = g_p2;

    cv::undistort(src_image, correct_image, cameraMatrix, distCoeffs);
}

/**
 * @brief RemoveSmallRegion   去除小的连通域
 * @param InputImage          输入图像
 * @param OutputImage         输出图像
 * @param pixel               像素阈值
 */
void RemoveSmallRegion(cv::Mat& InputImage, cv::Mat& OutputImage, int threshold) {
    InputImage.copyTo(OutputImage);
    std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point>> contours;  // 检测到的轮廓

    cv::findContours(InputImage, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

    // 去除小轮廓，保留大轮廓-----hierarchy[i][0]即i轮廓的后一个轮廓对应的索引
    for (int i = 0; i >= 0; i = hierarchy[i][0]) {
        double contourArea = fabs(cv::contourArea(contours[i]));

        // 轮廓面积小于指定值，则用黑色将其填充
        if (contourArea < threshold) {
            // qDebug() << "轮廓" << i << "的大小为" << contourArea << ",小于阈值" << threshold;
            cv::drawContours(OutputImage, contours, i, cv::Scalar(0), cv::FILLED, cv::LINE_8,
                             hierarchy);
        }
    }
}

/**
 * @brief GrayCenter_horizontal  灰度重心法（激光条纹水平）
 * @param InputImage			 输入图像
 * @param Pt					 中心点坐标值
 * @param bounding_rect			 连通域轮廓外接矩形
 * @param threshold				 灰度阈值
 */
void GrayCenterHorizontal(cv::Mat& InputImage, std::vector<cv::Point2f>& Pt, cv::Rect bounding_rect,
                          int threshold) {
    std::vector<cv::Point2f> P;
    for (int i = bounding_rect.x; i < bounding_rect.x + bounding_rect.width; ++i) {
        int sum = 0;  // 每列灰度值的和
        float y = 0;  // 每列中心点纵坐标
        for (int j = bounding_rect.y; j < bounding_rect.y + bounding_rect.height; ++j) {
            int s = InputImage.at<uchar>(j, i);
            if (s) {
                sum += s;
                y += j * s;
            }
        }
        if (sum) {
            y /= sum;
            if (InputImage.at<uchar>(y, i) > 0) {
                P.emplace_back(cv::Point2f(i, y));
            }
        }
    }

    // 对中心线上的点进行平滑滤波
    if (P.size() >= 3) {
        for (size_t i = 1; i < P.size() - 1; ++i) {
            P[i].y = (P[i - 1].y + P[i].y + P[i + 1].y) / 3;
        }
    }

    // 计算中心线上点的平均灰度值
    if (P.size() > 0) {
        int avg_scalar = 0;
        for (size_t i = 0; i < P.size(); ++i) {
            avg_scalar += InputImage.at<uchar>(round(P[i].y), round(P[i].x));
        }
        avg_scalar /= P.size();

        if (avg_scalar < threshold) P.clear();
    }

    // 去除中心线上灰度值过低的点
    for (size_t i = 0; i < P.size(); ++i) {
        if (P[i].x >= 0 && P[i].x < InputImage.cols && P[i].y >= 0 && P[i].y < InputImage.rows) {
            if (InputImage.at<uchar>(round(P[i].y), round(P[i].x)) > threshold) {
                Pt.emplace_back(P[i]);
            }
        }
    }
}

/**
 * @brief CenterLine_horizontal  提取中心线（激光条纹水平）
 * @param correct_image			 校正后的图像
 * @param dst_image				 处理结果图
 * @param Pt				     中心点坐标值
 */
void CenterLineHorizontal(int m, cv::Mat& correct_image, cv::Mat& dst_image,
                          std::vector<cv::Point2f>& Pt) {
    dst_image = correct_image.clone();

    // ROI提取（从(0,0)开始，取宽度为correct_image.cols，高度为500）
    //    correct_image = correct_image(cv::Rect(0, 0, correct_image.cols, 500));
    cv::Mat img1 = correct_image.clone();
    cv::Mat img2;

    cv::GaussianBlur(img1, img1, cv::Size(0, 0), g_sigmaX, g_sigmaY);  // 高斯滤波
    qDebug().nospace() << "g_sigmaX:" << g_sigmaX << "  g_sigmaY:" << g_sigmaY;

    cv::imwrite("./data/image/after0_Gauss_" + std::to_string((int)g_sigmaX) + ".bmp", img1);

    // 求每列灰度值最大值
    uchar* p = img1.data;
    std::vector<int> max_col_scalar(img1.cols);

    for (int i = 0; i < img1.cols; ++i) {
        for (int j = 0; j < img1.rows; ++j) {
            if (*(p + i + img1.cols * j) > max_col_scalar[i]) {
                max_col_scalar[i] = *(p + i + img1.cols * j);
            }
        }
    }

    // 按列阈值操作
    int pixels = 0;
    p = img1.data;
    qDebug().nospace() << "g_offset:" << g_offset;
    for (int i = 0; i < img1.cols; ++i) {
        //        int threshold = std::max(max_col_scalar[i] - (float)30, g_minThreshold);
        int threshold = std::max(max_col_scalar[i] - g_offset, g_minThreshold);

        for (int j = 0; j < img1.rows; ++j) {
            if (*(p + i + img1.cols * j) < threshold)
                *(p + i + img1.cols * j) = 0;
            else
                ++pixels;
        }
    }
    cv::imwrite("./data/image/after1_threshold_" + std::to_string((int)g_offset) + ".bmp", img1);

    //    cv::GaussianBlur(img1, img1, cv::Size(0, 0), 6, 6);  // 高斯滤波

    //    if (pixels < 1000) return;
    RemoveSmallRegion(img1, img2, g_region_minArea);  // 面积滤波
    cv::imwrite(
        "./data/image/after2_RemoveSmallRegion_" + std::to_string((int)g_region_minArea) + ".bmp",
        img2);

    std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point>> contours;  // 连通域轮廓
    cv::findContours(img2, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    std::vector<cv::Rect> bounding_rect;  // 储存连通域轮廓外接矩形
    for (size_t i = 0; i < contours.size(); ++i) {
        bounding_rect.emplace_back(cv::boundingRect(cv::Mat(contours[i])));
    }
    //    qDebug() << "bounding_rect_size:" << bounding_rect.size() << "->" << bounding_rect[0].x
    //             << bounding_rect[0].y << bounding_rect[0].width << bounding_rect[0].height;

    std::vector<cv::Point2f> centerPoint;  // 存储每个区域提取出的中心点坐标

    cv::Mat img5;
    for (size_t i = 0; i < contours.size(); ++i) {
        cv::Mat img3 = img2.clone();
        // 轮廓用黑色填充，相当于去除连通域
        cv::drawContours(img3, contours, i, cv::Scalar(0), cv::FILLED, 8, hierarchy);
        cv::Mat img4 = img2 - img3;

        cv::imwrite("./data/image/after3_minus" + std::to_string(i) + ".bmp", img4);

        // ============================== 形态学操作尝试 ==============================
        // 创建结构元素（内核）
        //        int erosion_size = 1;
        //        cv::Mat element = cv::getStructuringElement(
        //            cv::MORPH_RECT, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
        //            cv::Point(erosion_size, erosion_size));
        //        cv::dilate(img4, img4, element);
        //        cv::erode(img4, img4, element);
        //        cv::erode(img4, img4, element);
        //        cv::dilate(img4, img4, element);
        //        cv::dilate(img4, img4, element);
        //        cv::erode(img4, img4, element);
        //        cv::imwrite("./data/image/after4_morphological_" + std::to_string(i) + ".bmp",
        //        img4);

        // ============================== 形态学操作尝试 ==============================

        cv::GaussianBlur(img4, img4, cv::Size(0, 0), 1.5);  // 高斯滤波
        cv::imwrite("./data/image/after5_Gauss2_" + std::to_string(i) + ".bmp", img4);
        img5 = img4;

        // 灰度重心法提取中心线
        GrayCenterHorizontal(img4, centerPoint, bounding_rect[i], g_minThreshold / 2);

        img2 = img3;
    }

    cv::cvtColor(dst_image, dst_image, cv::COLOR_GRAY2RGB);
    Pt.clear();
    for (size_t i = 0; i < centerPoint.size(); ++i) {
        Pt.emplace_back(cv::Point2f(centerPoint[i].x, centerPoint[i].y));
        cv::circle(dst_image, cv::Point(round(centerPoint[i].x), round(centerPoint[i].y)), 0,
                   cv::Scalar(0, 0, 255));  // 画出中心线
    }
    cv::imwrite("./data/center/" + std::to_string(m) + ".bmp", dst_image);

    //    cv::cvtColor(img5, img5, cv::COLOR_GRAY2RGB);
    //    Pt.clear();
    //    for (size_t i = 0; i < centerPoint.size(); ++i) {
    //        Pt.emplace_back(cv::Point2f(centerPoint[i].x, centerPoint[i].y));
    //        cv::circle(img5, cv::Point(round(centerPoint[i].x), round(centerPoint[i].y)), 0,
    //                   cv::Scalar(0, 0, 255));  // 画出中心线
    //    }
    //    cv::imwrite("./data/center/" + std::to_string(m) + ".bmp", img5);
    //    dst_image = img5;

    m++;
}

/**
 * @brief GrayCenter_vertical    灰度重心法（激光条纹垂直）
 * @param InputImage				输入图像
 * @param Pt						中心点坐标值
 * @param bounding_rect			连通域轮廓外接矩形
 * @param threshold				灰度阈值
 */
void GrayCenterVertical(cv::Mat& InputImage, std::vector<cv::Point2f>& Pt, cv::Rect bounding_rect,
                        int threshold) {
    std::vector<cv::Point2f> P;
    for (int i = bounding_rect.y; i < bounding_rect.y + bounding_rect.height; ++i) {
        int sum = 0;  // 每列灰度值的和
        float x = 0;  // 每列中心点纵坐标
        for (int j = bounding_rect.x; j < bounding_rect.x + bounding_rect.width; ++j) {
            int s = InputImage.at<uchar>(i, j);
            if (s) {
                sum += s;
                x += j * s;
            }
        }
        if (sum) {
            x /= sum;
            if (InputImage.at<uchar>(i, x) > 0) {
                P.emplace_back(cv::Point2f(x, i));
            }
        }
    }

    // 对中心线上的点进行平滑滤波
    if (P.size() >= 3) {
        for (size_t i = 1; i < P.size() - 1; ++i) {
            P[i].x = (P[i - 1].x + P[i].x + P[i + 1].x) / 3;
        }
    }

    // 计算中心线上点的平均灰度值
    if (P.size() > 0) {
        int avg_scalar = 0;
        for (size_t i = 0; i < P.size(); ++i) {
            avg_scalar += InputImage.at<uchar>(round(P[i].y), round(P[i].x));
        }
        avg_scalar /= P.size();

        if (avg_scalar < threshold) P.clear();
    }

    // 去除中心线上灰度值过低的点
    for (size_t i = 0; i < P.size(); ++i) {
        if (P[i].x >= 0 && P[i].x < InputImage.cols && P[i].y >= 0 && P[i].y < InputImage.rows) {
            if (InputImage.at<uchar>(round(P[i].y), round(P[i].x)) > threshold) {
                Pt.emplace_back(P[i]);
            }
        }
    }
}

/**
 * @brief CenterLine_vertical    提取中心线（激光条纹垂直）
 * @param correct_image			校正后的图像
 * @param dst_image				处理结果图
 * @param Pt						中心点坐标值
 */
void CenterLineVertical(cv::Mat& correct_image, cv::Mat& dst_image, std::vector<cv::Point2f>& Pt) {
    dst_image = correct_image.clone();
    // ROI提取（从(0,0)开始，取宽度为correct_image.cols，高度为500）
    //    correct_image = correct_image(cv::Rect(0, 0, correct_image.cols, 500));
    cv::Mat img1 = correct_image.clone(), img2;

    cv::GaussianBlur(img1, img1, cv::Size(0, 0), g_sigmaX,
                     g_sigmaX);  // 高斯滤波

    // 求每行灰度值最大值
    uchar* p = img1.data;
    std::vector<int> max_row_scalar(img1.rows);
    for (int i = 0; i < img1.rows; ++i) {
        for (int j = 0; j < img1.cols; ++j) {
            if (*(p + img1.cols * i + j) > max_row_scalar[i]) {
                max_row_scalar[i] = *(p + img1.cols * i + j);
            }
        }
    }

    // 按行阈值操作
    int pixels = 0;
    p = img1.data;
    for (int i = 0; i < img1.rows; ++i) {
        int threshold = std::max(max_row_scalar[i] - g_offset, g_minThreshold);
        for (int j = 0; j < img1.cols; ++j) {
            if (*(p + img1.cols * i + j) < threshold) {
                *(p + img1.cols * i + j) = 0;
            } else {
                ++pixels;
            }
        }
    }
    // if (pixels < 1000)    return;

    RemoveSmallRegion(img1, img2, g_region_minArea);  // 面积滤波

    std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point>> contours;  // 连通域轮廓
    cv::findContours(img2, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    std::vector<cv::Rect> bounding_rect;  // 储存连通域轮廓外接矩形
    for (size_t i = 0; i < contours.size(); ++i) {
        bounding_rect.emplace_back(cv::boundingRect(cv::Mat(contours[i])));
    }

    std::vector<cv::Point2f> P;  // 存储每个区域提取出的中心点坐标
    for (size_t i = 0; i < contours.size(); ++i) {
        cv::Mat img3 = img2.clone();
        cv::drawContours(img3, contours, i, cv::Scalar(0), cv::FILLED, 8,
                         hierarchy);  // 轮廓用黑色填充，相当于去除连通域
        cv::Mat img4 = img2 - img3;
        GrayCenterVertical(img4, P, bounding_rect[i],
                           g_minThreshold / 2);  // 灰度重心法提取中心线
        img2 = img3;
    }

    // cv::cvtColor(dst_image, dst_image, cv::COLOR_GRAY2RGB);

    for (size_t i = 0; i < P.size(); ++i) {
        Pt.emplace_back(cv::Point2f(P[i].x, P[i].y));
        // cv::circle(dst_image, cv::Point(round(P[i].x), round(P[i].y)), 1, cv::Scalar(0, 0, 255),
        // -1); //画出中心线
    }
}

/**
* @brief Point2dto3d        二维坐标转三维坐标
* @param n					图像序号
* @param Pt					中心点坐标值

*/
void Point2dto3d(int n, std::vector<cv::Point2f>& Pt, pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloud) {
    if (Pt.size() == 0) {
        std::cout << "Pt=0" << std::endl;
        return;
    }
    ptcloud->points.resize(Pt.size());

    //    float u = g_u, v = g_v, w = g_w, x = g_x, y = g_y, z = g_z;

    for (size_t i = 0; i < Pt.size(); ++i) {
        float u0 = g_u0, v0 = g_v0;  // 相机内参
        float fx = g_fx, fy = g_fy;  // 相机内参

        float u = Pt[i].x, v = Pt[i].y;                // 像素坐标
        float xp = (u - u0) / fx, yp = (v - v0) / fy;  // 图像坐标
        float a = g_a, b = g_b,
              c = g_c;  // 光平面方程系数

        cv::Mat_<float> Camera = cv::Mat::ones(4, 1, CV_32FC1);  // 相机坐标系下三维轮廓点坐标
        Camera.at<float>(0, 0) = xp / (a * xp + b * yp + c);
        Camera.at<float>(1, 0) = yp / (a * xp + b * yp + c);
        Camera.at<float>(2, 0) = 1 / (a * xp + b * yp + c);

        ptcloud->points[i] =
            pcl::PointXYZ(Camera.at<float>(0, 0), Camera.at<float>(1, 0), Camera.at<float>(2, 0));
    }

    pcl::io::savePCDFileBinary("./data/cloud/" + std::to_string(n) + ".pcd",
                               *ptcloud);  // 存储.pcd格式点云文件
}

/**
 * @brief Point2dto3dRotaion 二维坐标转三维坐标（旋转）
 * @param n					图像序号
 * @param Pt					中心点坐标值
 * @param angular_velocity   转动角速度
 * @param times				扫描计时
 */
void Point2dto3dRotaion(int n, std::vector<cv::Point2f>& Pt, std::vector<float> angles) {
    if (Pt.size() == 0) {
        std::cout << "Pt=0" << std::endl;
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloud(
        new pcl::PointCloud<pcl::PointXYZ>);  // 点云轮廓的三维坐标
    ptcloud->points.resize(Pt.size());

    float u = g_u, v = g_v, w = g_w, x = g_x, y = g_y, z = g_z;
    float t = angles[n] * M_PI / 180;
    cv::Mat matrix =
        (cv::Mat_<float>(4, 4) << u * u + (v * v + w * w) * cos(t),
         u * v * (1 - cos(t)) - w * sin(t), u * w * (1 - cos(t)) + v * sin(t),
         (x * (v * v + w * w) - u * (y * v + z * w)) * (1 - cos(t)) + (y * w - z * v) * sin(t),
         u * v * (1 - cos(t)) + w * sin(t), v * v + (u * u + w * w) * cos(t),
         v * w * (1 - cos(t)) - u * sin(t),
         (y * (u * u + w * w) - v * (x * u + z * w)) * (1 - cos(t)) + (z * u - x * w) * sin(t),
         u * w * (1 - cos(t)) - v * sin(t), v * w * (1 - cos(t)) + u * sin(t),
         w * w + (u * u + v * v) * cos(t),
         (z * (u * u + v * v) - w * (x * u + y * v)) * (1 - cos(t)) + (x * v - y * u) * sin(t), 0,
         0, 0, 1);

    for (size_t i = 0; i < Pt.size(); ++i) {
        float u0 = g_u0, v0 = g_v0;  // 相机内参
        float fx = g_fx, fy = g_fy;  // 相机内参

        float u = Pt[i].x, v = Pt[i].y;                // 像素坐标
        float xp = (u - u0) / fx, yp = (v - v0) / fy;  // 图像坐标
        float a = g_a, b = g_b,
              c = g_c;  // 光平面方程系数

        cv::Mat_<float> Camera = cv::Mat::ones(4, 1, CV_32FC1);  // 相机坐标系下三维轮廓点坐标
        Camera.at<float>(0, 0) = xp / (a * xp + b * yp + c);
        Camera.at<float>(1, 0) = yp / (a * xp + b * yp + c);
        Camera.at<float>(2, 0) = 1 / (a * xp + b * yp + c);

        Camera = matrix * Camera;

        ptcloud->points[i] =
            pcl::PointXYZ(Camera.at<float>(0, 0), Camera.at<float>(1, 0), Camera.at<float>(2, 0));
    }

    pcl::io::savePCDFileBinary("./data/cloud/" + std::to_string(n) + ".pcd",
                               *ptcloud);  // 存储.pcd格式点云文件
}

/**
 * @brief Point2dto3dLinear 二维坐标转三维坐标（线性）
 * @param n					图像序号
 * @param Pt					中心点坐标值
 * @param angular_velocity   转动角速度
 * @param times				扫描计时
 */
void Point2dto3dLinear(int n, std::vector<cv::Point2f>& Pt, std::vector<float> moves,
                       std::vector<float> dir) {
    float move0 = moves[0];
    float d_move = moves[n] - move0;
    std::cout << " moves " << n << " " << moves[n] << " " << move0 << " " << moves[n] - move0
              << std::endl;
    // std::cout<<"dir"<<dir[0]<<" "<<dir[1]<<" "<<dir[2]<<std::endl;

    if (Pt.size() == 0) {
        std::cout << "Pt=0" << std::endl;
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloud(
        new pcl::PointCloud<pcl::PointXYZ>);  // 点云轮廓的三维坐标
    ptcloud->points.resize(Pt.size());

    cv::Mat matrix = (cv::Mat_<float>(4, 4) << 1, 0, 0, d_move * abs(dir[0]), 0, 1, 0,
                      d_move * abs(dir[1]), 0, 0, 1, d_move * abs(dir[2]), 0, 0, 0, 1);
    // pt生成点云
    for (size_t i = 0; i < Pt.size(); ++i) {
        float u0 = g_u0, v0 = g_v0;  // 相机内参
        float fx = g_fx, fy = g_fy;  // 相机内参

        float u = Pt[i].x, v = Pt[i].y;                // 像素坐标
        float xp = (u - u0) / fx, yp = (v - v0) / fy;  // 图像坐标
        float a = g_a, b = g_b,
              c = g_c;  // 光平面方程系数

        cv::Mat_<float> Camera = cv::Mat::ones(4, 1, CV_32FC1);  // 相机坐标系下三维轮廓点坐标
        Camera.at<float>(0, 0) = xp / (a * xp + b * yp + c);
        Camera.at<float>(1, 0) = yp / (a * xp + b * yp + c);
        Camera.at<float>(2, 0) = 1 / (a * xp + b * yp + c);

        Camera = matrix * Camera;

        ptcloud->points[i] =
            pcl::PointXYZ(Camera.at<float>(0, 0), Camera.at<float>(1, 0), Camera.at<float>(2, 0));
    }
    pcl::io::savePCDFileBinary("./data/cloud/" + std::to_string(n) + ".pcd",
                               *ptcloud);  // 存储.pcd格式点云文件
}

std::string image_path;
std::string cloud_path;
std::vector<float> coeffcient;

float g_fx;
float g_fy;
float g_u0;
float g_v0;

float g_k1;
float g_k2;
float g_p1;
float g_p2;

float g_sigmaX;
float g_sigmaY;
float g_minThreshold;
float g_offset;
float g_region_minArea;

float g_a;
float g_b;
float g_c;

float g_u;
float g_v;
float g_w;
float g_x;
float g_y;
float g_z;

}  // namespace ImageProcessing
