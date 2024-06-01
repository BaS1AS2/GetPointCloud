#include "camera_calibration.h"

#define BOARD_SCALE 3   // 棋盘格边长（mm）
#define BOARD_HEIGHT 8  // 棋盘格高度方向角点个数
#define BOARD_WIDTH 11  // 棋盘格宽度方向角点个数

/**
 * @brief CamraCalibration  相机标定
 * @param files             文件名
 * @param cameraMatrix      内参矩阵
 * @param distCoeffs        畸变系数
 * @param tvecsMat          平移矩阵
 * @param rvecsMat          旋转矩阵
 */
void Calibration::CamraCalibration(std::vector<std::string>& files,
                                   cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
                                   std::vector<cv::Mat>& tvecsMat,
                                   std::vector<cv::Mat>& rvecsMat) {
    // 读取每一幅图像，从中提取出角点，然后对角点进行亚像素精确化
    int image_count = 0;  // 图像数量
    cv::Size image_size;  // 图像的尺寸
    cv::Size board_size =
        cv::Size(BOARD_HEIGHT, BOARD_WIDTH);  // 标定板上每行、列的角点数
    std::vector<cv::Point2f> image_points_buf;  // 缓存每幅图像上检测到的角点
    std::vector<std::vector<cv::Point2f>>
        image_points_seq;  // 保存检测到的所有角点

    for (int i = 0; i < files.size(); i++) {
        cv::Mat imageInput = cv::imread(files[i]);

        /* 提取角点 */
        // imageInput:输入图片, board_size:角点尺寸, image_points_buf:检测结果
        if (0 == cv::findChessboardCorners(imageInput, board_size,
                                           image_points_buf)) {
            std::cout << "Num " << i
                      << " can not find chessboard corners!\n";  // 找不到角点
            continue;
        } else {
            // 找到一幅有效的图片
            image_count++;
            if (image_count == 1) {  // 读入第一张图片时获取图像宽高信息
                image_size.width = imageInput.cols;
                image_size.height = imageInput.rows;
            }

            cv::Mat view_gray;
            cv::cvtColor(imageInput, view_gray, cv::COLOR_RGB2GRAY);

            /* 亚像素精确化 */
            // 对已经检测到的角点进行亚像素优化，提供已经检测到的角点，搜索区域大小5*5，不适用零区域，迭代30次或精度达到0.1后停止
            cv::cornerSubPix(view_gray, image_points_buf, cv::Size(5, 5),
                             cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::MAX_ITER +
                                                  cv::TermCriteria::EPS,
                                              30, 0.1));
            // 统一角点顺序
            if (image_points_buf[0].x >
                image_points_buf[image_points_buf.size() - 1].x) {
                std::vector<cv::Point2f> buf;
                for (int i = 0; i < image_points_buf.size(); i++) {
                    buf.push_back(
                        image_points_buf[image_points_buf.size() - 1 - i]);
                }
                image_points_buf.clear();
                image_points_buf = buf;
            }

            image_points_seq.push_back(image_points_buf);  // 保存亚像素角点
            int a = 0;
            for (int j = 0; j < image_points_buf.size(); j++) {
                cv::circle(
                    imageInput,
                    cv::Point(image_points_buf[j].x, image_points_buf[j].y), 1,
                    cv::Scalar(0, 0, 255), -1);  // 画圆
                cv::Point text_position(image_points_buf[j].x,
                                        image_points_buf[j].y);
                int font_face = cv::FONT_HERSHEY_SIMPLEX;
                double font_scale = 0.5;
                cv::Scalar font_color(255, 0, 0);  // 文本颜色，BGR格式
                int font_thickness = 1;
                cv::putText(imageInput, std::to_string(a++), text_position,
                            font_face, font_scale, font_color, font_thickness);
            }
            // 保存角点图片
            std::string outputFilename = std::to_string(i) + "output_image.bmp";
            bool result = cv::imwrite(outputFilename, imageInput);
            if (!result) {
                std::cerr << "Failed to save image." << std::endl;
                return;
            }
        }
    }

    //    int total = image_points_seq.size();
    // std::cout << "Total num is" << total << std::endl;
    // std::cout << "get corner suc";

    /* 棋盘三维信息 */
    cv::Size2f square_size = cv::Size2f(
        BOARD_SCALE, BOARD_SCALE);  // 实际测量得到的标定板上每个棋盘格的大小
    std::vector<std::vector<cv::Point3f>>
        object_points;  // 保存标定板上角点的三维坐标
    cameraMatrix =
        cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));  // 摄像机内参数矩阵
    std::vector<int> point_counts;  // 每幅图像中角点的数量
    distCoeffs = cv::Mat(1, 4, CV_32FC1,
                         cv::Scalar::all(0));  // 摄像机的畸变系数：k1,k2,p1,p2

    /* 初始化标定板上角点的三维坐标 */
    int i, j, t;
    for (t = 0; t < image_count; t++) {
        std::vector<cv::Point3f> tempPointSet;
        //        qDebug() << "board_size.height:" << board_size.height;
        //        qDebug() << "board_size.width:" << board_size.width;
        for (i = 0; i < board_size.height; i++) {
            for (j = 0; j < board_size.width; j++) {
                cv::Point3f realPoint;
                /* 假设标定板放在世界坐标系中z=0的平面上 */
                realPoint.y = i * square_size.width;
                realPoint.x = j * square_size.height;
                realPoint.z = 0;
                tempPointSet.push_back(realPoint);
                //                qDebug() << realPoint.x << "," << realPoint.y;
            }
        }
        object_points.push_back(tempPointSet);
    }
    // 打印角点顺序问题
    //    for (int j = 0; j < object_points.size(); j++)
    //    {
    //        int a = 0;
    //        cv::Mat imageInput = cv::imread(files[j]);
    //        for (int i = 0; i < object_points[j].size(); i++)
    //        {
    ////            qDebug() << object_points[j][i].x << "," <<
    /// object_points[j][i].y << "->" /                     <<
    /// image_points_seq[j][i].x << "," << image_points_seq[j][i].y;
    //            cv::Point text_position(image_points_seq[j][i].x,
    //            image_points_seq[j][i].y); int font_face =
    //            cv::FONT_HERSHEY_SIMPLEX; double font_scale = 0.5; cv::Scalar
    //            font_color(0, 0, 255);  // 文本颜色，BGR格式 int
    //            font_thickness = 1; cv::putText(imageInput, std::to_string(a),
    //            text_position, font_face,
    //                            font_scale, font_color, font_thickness);
    //            cv::Point text_position2(object_points[j][i].x * 8,
    //            object_points[j][i].y * 8); cv::Scalar font_color2(0, 255, 0);
    //            // 文本颜色，BGR格式 cv::putText(imageInput,
    //            std::to_string(a++), text_position2, font_face,
    //                            font_scale, font_color2, font_thickness);
    //        }
    //        std::string outputFilename = std::to_string(j) +
    //        "output_image.bmp"; bool result = cv::imwrite(outputFilename,
    //        imageInput); if (!result) {
    //            std::cerr << "Failed to save image." << std::endl;
    //            return;
    //        }
    //    }

    /* 初始化每幅图像中的角点数量，假定每幅图像中都可以看到完整的标定板 */
    for (i = 0; i < image_count; i++) {
        point_counts.push_back(board_size.width * board_size.height);
    }

    /* 开始标定 */
    /*double cv::calibrateCamera(
        InputArrayOfArrays objectPoints,     // 三维物体空间点的集合，类型为
    std::vector<std::vector<cv::Point3f>> InputArrayOfArrays imagePoints, //
    二维图像空间点的集合，类型为 std::vector<std::vector<cv::Point2f>> Size
    imageSize,                      // 图像尺寸，类型为 cv::Size(width, height)
        InputOutputArray cameraMatrix,       // 输出的相机矩阵，类型为 cv::Mat
        InputOutputArray distCoeffs,         // 输出的畸变系数，类型为 cv::Mat
        OutputArrayOfArrays rvecs,           // 输出的旋转向量，类型为
    std::vector<cv::Mat> OutputArrayOfArrays tvecs,           //
    输出的平移向量，类型为 std::vector<cv::Mat> int flags = 0, //
    标定选项，如是否固定某些参数 TermCriteria criteria =
    TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, DBL_EPSILON)  //
    迭代终止准则
    );*/
    cv::calibrateCamera(object_points, image_points_seq, image_size,
                        cameraMatrix, distCoeffs, rvecsMat, tvecsMat,
                        cv::CALIB_FIX_K3);
    //    cv::calibrateCamera(object_points, image_points_seq, image_size,
    //    cameraMatrix,
    //                        distCoeffs, rvecsMat, tvecsMat,
    //                        cv::CALIB_USE_INTRINSIC_GUESS & cv::CALIB_FIX_K3);

    // std::cout << "calibration succ" << std::endl;

    // 对标定结果进行评价
    /*
     * 个人理解：在进行标定时，相机是相对固定的，通过移动标定板并拍摄图片来得到不同位置姿态的标定板图片，
     * 但实际上，我们是认为标定板在空间中的三维坐标是固定不变的，通过移动相机来拍摄不同角度下的标定板。
     * 得到每张标定板角点在图像中的二维坐标后，我们进行了相机的标定，得到的是相机的内参（内参矩阵和畸变系数），
     * 以及相机的外参（假设标定板不动，相机移动前提下的相机的旋转平移矩阵）。
     * 通过得到的内外参矩阵以及固定的真实棋盘格三维空间角点坐标，我们可以得到这些三维点重新投影于对应的二维图片中应当的位置，
     * 从而将其与之前检测到的角点坐标进行比较，得到之前检测角点时的误差。
     * 存在的问题：我们需要使用角点检测的结果来计算相机的内外参数，然后又使用这些内外参数来进行重投影，最终用重投影结果来评价角点检测的质量。
     */
    double total_err = 0.0;  // 所有图像的平均误差的总和
    double err = 0.0;        // 每幅图像的平均误差
    std::vector<cv::Point2f> image_points2;  // 保存重新计算得到的投影点
    for (i = 0; i < image_count; i++) {
        std::vector<cv::Point3f> tempPointSet = object_points[i];
        /* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点
         */
        cv::projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix,
                          distCoeffs, image_points2);
        /* 计算新的投影点和旧的投影点之间的误差 */
        std::vector<cv::Point2f> tempImagePoint = image_points_seq[i];
        cv::Mat tempImagePointMat = cv::Mat(1, tempImagePoint.size(), CV_32FC2);
        cv::Mat image_points2Mat = cv::Mat(1, image_points2.size(), CV_32FC2);
        for (int j = 0; j < tempImagePoint.size(); j++) {
            image_points2Mat.at<cv::Vec2f>(0, j) =
                cv::Vec2f(image_points2[j].x, image_points2[j].y);
            tempImagePointMat.at<cv::Vec2f>(0, j) =
                cv::Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
        }
        err = cv::norm(image_points2Mat, tempImagePointMat, cv::NORM_L2);
        total_err += err /= point_counts[i];
    }
    // 保存定标结果
    cv::Mat rotation_matrix =
        cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));  // 保存每幅图像的旋转矩阵
    for (int i = 0; i < image_count; i++) {
        /* 将旋转向量转换为相对应的旋转矩阵 */
        cv::Rodrigues(rvecsMat[i], rotation_matrix);
    }
}

/**
 * @brief GetImage  取得原图
 * @param img_src   图像名称
 * @param src_image 原始图像
 */
void Calibration::GetImage(std::string img_src, cv::Mat& src_image) {
    src_image = cv::imread(img_src, 0);
}

/**
 * @brief Correction    畸变校正
 * @param src_image     原始图像
 * @param dst_image     结果图像
 * @param cameraMatrix  内参矩阵
 * @param distCoeffs    畸变系数
 */
void Calibration::Correction(cv::Mat& src_image, cv::Mat& dst_image,
                             cv::Mat& cameraMatrix, cv::Mat& distCoeffs) {
    // distCoeffs.at<double>(0, 4) = 0;

    undistort(src_image, dst_image, cameraMatrix, distCoeffs);
}

/**
 * @brief RemoveSmallRegion 去除面积小于给定值的连通域
 * @param InputImage        输入图像
 * @param OutputImage       输出图像
 * @param pixel             像素值
 */
void Calibration::RemoveSmallRegion(cv::Mat& InputImage, cv::Mat& OutputImage,
                                    int pixel) {
    InputImage.copyTo(OutputImage);
    std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(InputImage, contours, hierarchy, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_NONE);

    //计算每个轮廓的面积
    double temp = 0;
    for (int i = 0; i < contours.size(); ++i) {
        temp = fabs(cv::contourArea(contours[i]));
    }

    //去除小轮廓，保留大轮廓
    for (int idx = 0; idx >= 0; idx = hierarchy[idx][0]) {
        if (fabs(cv::contourArea(contours[idx])) < pixel) {
            cv::drawContours(OutputImage, contours, idx, cv::Scalar(0),
                             cv::FILLED, 8, hierarchy);
        }
    }
}

/**
 * @brief GrayCenter      灰度重心法提取中心线
 * @param InputImage      输入图像
 * @param Pt              中心点像素坐标
 * @param bounding_rect   连通域外接矩形
 * @param threshold       灰度阈值
 */
void Calibration::GrayCenter(cv::Mat& InputImage, std::vector<cv::Point2d>& Pt,
                             cv::Rect bounding_rect, int threshold) {
    std::vector<cv::Point2d> P;
    for (int i = bounding_rect.x; i < bounding_rect.x + bounding_rect.width;
         ++i) {
        int sum = 0;   // 每列灰度值的和
        double y = 0;  // 每列中心点纵坐标
        for (int j = bounding_rect.y;
             j < bounding_rect.y + bounding_rect.height; ++j) {
            int s = InputImage.at<uchar>(j, i);
            if (s) {
                sum += s;
                y += j * s;
            }
        }
        if (sum) {
            y /= sum;
            if (InputImage.at<uchar>(y, i) > 0) {
                P.emplace_back(cv::Point2d(i, y));
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
        if (P[i].x >= 0 && P[i].x < InputImage.cols && P[i].y >= 0 &&
            P[i].y < InputImage.rows) {
            if (InputImage.at<uchar>(round(P[i].y), round(P[i].x)) >
                threshold) {
                Pt.emplace_back(P[i]);
            }
        }
    }
}

// 对点的坐标按x从小到大排序，若x值相同则按y从小到大排序//
bool Calibration::PointSortRule(const cv::Point2d pt1, const cv::Point2d pt2) {
    if (pt1.x != pt2.x)
        return pt1.x < pt2.x;
    else
        return pt1.y < pt2.y;
}

// 亚像素级激光条纹中心线提取
void Calibration::CenterLine(int n, cv::Mat& correct_image,
                             std::vector<cv::Point2d>& P) {
    cv::Mat dst_image = correct_image.clone();
    cv::cvtColor(dst_image, dst_image, cv::COLOR_GRAY2RGB);
    cv::Mat img1 = correct_image.clone(), img2;

    GaussianBlur(img1, img1, cv::Size(0, 0), 1.1, 1.1);  // 高斯滤波

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
    p = img1.data;
    for (int i = 0; i < img1.cols; ++i) {
        // 如果所在列最大灰度值减去20后比100大，取阈值为列最大灰度值减20，否则取阈值为100
        int threshold = std::max(max_col_scalar[i] - 20, 100);
        // 小于阈值的像素都设为0
        for (int j = 0; j < img1.rows; ++j) {
            if (*(p + i + img1.cols * j) < threshold)
                *(p + i + img1.cols * j) = 0;
        }
    }

    RemoveSmallRegion(img1, img2, 10);  // 面积滤波

    std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point>> contours;  // 连通域轮廓
    findContours(img2, contours, hierarchy, cv::RETR_EXTERNAL,
                 cv::CHAIN_APPROX_NONE);
    std::vector<cv::Rect> bounding_rect;  // 储存连通域轮廓外接矩形
    for (size_t i = 0; i < contours.size(); ++i) {
        bounding_rect.emplace_back(cv::boundingRect(cv::Mat(contours[i])));
    }

    P.clear();
    for (size_t i = 0; i < contours.size(); ++i) {
        cv::Mat img3 = img2.clone();
        cv::drawContours(img3, contours, i, cv::Scalar(0), cv::FILLED, 8,
                         hierarchy);  // 轮廓用黑色填充，相当于去除连通域
        cv::Mat img4 = img2 - img3;
        GrayCenter(img4, P, bounding_rect[i], 50);  // 灰度重心法提取中心线
        img2 = img3;
    }
    std::sort(P.begin(), P.end(), PointSortRule);

    std::ofstream fout("./calibration/" + std::to_string(n) + "_2d.txt");
    for (int i = 0; i < P.size(); i++) {
        fout << P[i].x << " " << P[i].y << std::endl;
        cv::circle(dst_image, cv::Point(round(P[i].x), round(P[i].y)), 0.5,
                   cv::Scalar(0, 0, 255), -1);  // 画出中心线
    }
    fout.close();
    cv::imwrite("calibration//C" + std::to_string(n) + ".bmp", dst_image);
}

/**
 * @brief GrayCenter_vertical    灰度重心法（激光条纹垂直）
 * @param InputImage				输入图像
 * @param Pt						中心点坐标值
 * @param bounding_rect			连通域轮廓外接矩形
 * @param threshold				灰度阈值
 */
void Calibration::GrayCenterVertical(cv::Mat& InputImage,
                                     std::vector<cv::Point2d>& Pt,
                                     cv::Rect bounding_rect, int threshold) {
    std::vector<cv::Point2d> P;
    for (int i = bounding_rect.y; i < bounding_rect.y + bounding_rect.height;
         ++i) {
        int sum = 0;  // 每行灰度值的和
        float x = 0;  // 每行中心点纵坐标
        for (int j = bounding_rect.x; j < bounding_rect.x + bounding_rect.width;
             ++j) {
            int s = InputImage.at<uchar>(i, j);
            if (s) {
                sum += s;
                x += j * s;
            }
        }
        if (sum) {
            x /= sum;
            if (InputImage.at<uchar>(i, x) > 0) {
                P.emplace_back(cv::Point2d(x, i));
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
        if (P[i].x >= 0 && P[i].x < InputImage.cols && P[i].y >= 0 &&
            P[i].y < InputImage.rows) {
            if (InputImage.at<uchar>(round(P[i].y), round(P[i].x)) >
                threshold) {
                Pt.emplace_back(P[i]);
            }
        }
    }
}

/**
 * @brief GrayCenter_horizontal  灰度重心法（激光条纹水平）
 * @param InputImage				输入图像
 * @param Pt						中心点坐标值
 * @param bounding_rect			连通域轮廓外接矩形
 * @param threshold				灰度阈值
 */
void Calibration::GrayCenterHorizontal(cv::Mat& InputImage,
                                       std::vector<cv::Point2d>& Pt,
                                       cv::Rect bounding_rect, int threshold) {
    std::vector<cv::Point2d> P;
    for (int i = bounding_rect.x; i < bounding_rect.x + bounding_rect.width;
         ++i) {
        int sum = 0;  //每列灰度值的和
        float y = 0;  //每列中心点纵坐标
        for (int j = bounding_rect.y;
             j < bounding_rect.y + bounding_rect.height; ++j) {
            int s = InputImage.at<uchar>(j, i);
            if (s) {
                sum += s;
                y += j * s;
            }
        }
        if (sum) {
            y /= sum;
            if (InputImage.at<uchar>(y, i) > 0) {
                P.emplace_back(cv::Point2d(i, y));
            }
        }
    }

    //对中心线上的点进行平滑滤波
    if (P.size() >= 3) {
        for (size_t i = 1; i < P.size() - 1; ++i) {
            P[i].y = (P[i - 1].y + P[i].y + P[i + 1].y) / 3;
        }
    }

    //计算中心线上点的平均灰度值
    if (P.size() > 0) {
        int avg_scalar = 0;
        for (size_t i = 0; i < P.size(); ++i) {
            avg_scalar += InputImage.at<uchar>(round(P[i].y), round(P[i].x));
        }
        avg_scalar /= P.size();

        if (avg_scalar < threshold) P.clear();
    }

    //去除中心线上灰度值过低的点
    for (size_t i = 0; i < P.size(); ++i) {
        if (P[i].x >= 0 && P[i].x < InputImage.cols && P[i].y >= 0 &&
            P[i].y < InputImage.rows) {
            if (InputImage.at<uchar>(round(P[i].y), round(P[i].x)) >
                threshold) {
                Pt.emplace_back(P[i]);
            }
        }
    }
}

/**
 * @brief CenterLine_horizontal  提取中心线（激光条纹水平）
 * @param correct_image			校正后的图像
 * @param dst_image				处理结果图
 * @param Pt						中心点坐标值
 */
void Calibration::CenterLineHorizontal(int m, cv::Mat& correct_image,
                                       cv::Mat& dst_image,
                                       std::vector<cv::Point2d>& Pt) {
    dst_image = correct_image.clone();
    // correct_image = correct_image(cv::Rect(0, 0, correct_image.cols, 500));
    cv::Mat img1 = correct_image.clone(), img2;

    cv::GaussianBlur(img1, img1, cv::Size(0, 0), 2, 2);  //高斯滤波

    //求每列灰度值最大值
    uchar* p = img1.data;
    std::vector<int> max_col_scalar(img1.cols);
    for (int i = 0; i < img1.cols; ++i) {
        for (int j = 0; j < img1.rows; ++j) {
            if (*(p + i + img1.cols * j) > max_col_scalar[i]) {
                max_col_scalar[i] = *(p + i + img1.cols * j);
            }
        }
    }

    //按列阈值操作
    int pixels = 0;
    p = img1.data;
    for (int i = 0; i < img1.cols; ++i) {
        int threshold = std::max(max_col_scalar[i] - 20, 100);
        for (int j = 0; j < img1.rows; ++j) {
            if (*(p + i + img1.cols * j) < threshold)
                *(p + i + img1.cols * j) = 0;
            else
                ++pixels;
        }
    }
    // if (pixels < 1000)    return;

    RemoveSmallRegion(img1, img2, 10);  //面积滤波

    std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point>> contours;  //连通域轮廓
    cv::findContours(img2, contours, hierarchy, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_NONE);
    std::vector<cv::Rect> bounding_rect;  //储存连通域轮廓外接矩形
    for (size_t i = 0; i < contours.size(); ++i) {
        bounding_rect.emplace_back(cv::boundingRect(cv::Mat(contours[i])));
    }

    std::vector<cv::Point2d> P;  //存储每个区域提取出的中心点坐标

    for (size_t i = 0; i < contours.size(); ++i) {
        cv::Mat img3 = img2.clone();
        cv::drawContours(img3, contours, i, cv::Scalar(0), cv::FILLED, 8,
                         hierarchy);  //轮廓用黑色填充，相当于去除连通域
        cv::Mat img4 = img2 - img3;
        GrayCenterHorizontal(img4, P, bounding_rect[i],
                             100 / 2);  //灰度重心法提取中心线
        img2 = img3;
    }

    for (size_t i = 0; i < P.size(); ++i) {
        Pt.emplace_back(cv::Point2d(P[i].x, P[i].y));
        // cv::circle(dst_image, cv::Point(round(P[i].x), round(P[i].y)), 1,
        // cv::Scalar(0, 0, 255), -1); //画出中心线
    }

    std::ofstream fout("calibration//" + std::to_string(m) + "_2d.txt");
    for (int i = 0; i < P.size(); i++) {
        fout << Pt[i].x << " " << Pt[i].y << std::endl;
        cv::circle(dst_image, cv::Point(round(P[i].x), round(P[i].y)), 0.5,
                   cv::Scalar(0, 0, 255), -1);  //画出中心线
    }
    fout.close();
    cv::imwrite("calibration//" + std::to_string(m) + "_2d.bmp", dst_image);
}

/**
 * @brief GrayCenter_vertical    灰度重心法（激光条纹垂直）
 * @param InputImage				输入图像
 * @param Pt						中心点坐标值
 * @param bounding_rect			连通域轮廓外接矩形
 * @param threshold				灰度阈值
 */
void Calibration::CenterLineVertical(int i, cv::Mat& correct_image,
                                     cv::Mat& dst_image,
                                     std::vector<cv::Point2d>& Pt) {
    dst_image = correct_image.clone();
    // correct_image = correct_image(cv::Rect(1200, 650, 77, 877));
    cv::Mat img1 = correct_image.clone(), img2;

    cv::GaussianBlur(img1, img1, cv::Size(0, 0), 2, 2);  //高斯滤波

    //求每行灰度值最大值
    uchar* p = img1.data;
    std::vector<int> max_row_scalar(img1.rows);
    for (int i = 0; i < img1.rows; ++i) {
        for (int j = 0; j < img1.cols; ++j) {
            if (*(p + img1.cols * i + j) > max_row_scalar[i]) {
                max_row_scalar[i] = *(p + img1.cols * i + j);
            }
        }
    }

    //按行阈值操作
    int pixels = 0;
    p = img1.data;
    for (int i = 0; i < img1.rows; ++i) {
        int threshold = std::max(max_row_scalar[i] - 20, 100);
        for (int j = 0; j < img1.cols; ++j) {
            if (*(p + img1.cols * i + j) < threshold) {
                *(p + img1.cols * i + j) = 0;
            } else {
                ++pixels;
            }
        }
    }
    // if (pixels < 1000)    return;

    RemoveSmallRegion(img1, img2, 10);  // 面积滤波

    std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point>> contours;  // 连通域轮廓
    cv::findContours(img2, contours, hierarchy, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_NONE);
    std::vector<cv::Rect> bounding_rect;  // 储存连通域轮廓外接矩形
    for (size_t i = 0; i < contours.size(); ++i) {
        bounding_rect.emplace_back(cv::boundingRect(cv::Mat(contours[i])));
    }

    std::vector<cv::Point2d> P;  // 存储每个区域提取出的中心点坐标
    for (size_t i = 0; i < contours.size(); ++i) {
        cv::Mat img3 = img2.clone();
        cv::drawContours(img3, contours, i, cv::Scalar(0), cv::FILLED, 8,
                         hierarchy);  // 轮廓用黑色填充，相当于去除连通域
        cv::Mat img4 = img2 - img3;
        GrayCenterVertical(img4, P, bounding_rect[i],
                           100 / 2);  // 灰度重心法提取中心线
        img2 = img3;
    }

    // cv::cvtColor(dst_image, dst_image, cv::COLOR_GRAY2RGB);

    for (size_t i = 0; i < P.size(); ++i) {
        Pt.emplace_back(cv::Point2d(P[i].x, P[i].y));
        // cv::circle(dst_image, cv::Point(round(P[i].x), round(P[i].y)), 1,
        // cv::Scalar(0, 0, 255), -1);  // 画出中心线
    }

    std::ofstream fout("calibration//" + std::to_string(i) + "_2d.txt");
    for (int i = 0; i < P.size(); i++) {
        fout << Pt[i].x << " " << Pt[i].y << std::endl;
        cv::circle(dst_image, cv::Point(round(P[i].x), round(P[i].y)), 0.5,
                   cv::Scalar(0, 0, 255), -1);  // 画出中心线
    }
    fout.close();
    cv::imwrite("calibration//" + std::to_string(i) + "_2d.bmp", dst_image);
}

/**
 * @brief Point2dSperate    分离坐标点
 * @param P                 中心点坐标值
 * @param P_plane           平面上的中心点
 * @param P_object          物体上的中心点
 */
void Calibration::Point2dSperate(std::vector<cv::Point2d>& P,
                                 std::vector<cv::Point2d>& P_plane,
                                 std::vector<cv::Point2d>& P_object) {
    P_plane.clear();
    P_object.clear();

    int object_begin1, object_end1;
    for (int i = 1; i < P.size(); ++i) {
        if (P[i - 1].y - P[i].y > 10) object_begin1 = i;
        if (P[i].y - P[i - 1].y > 10) object_end1 = i;
    }

    int object_begin2, object_end2;
    for (int i = 1; i < P.size(); ++i) {
        if (P[i - 1].y - P[i].y > 10) {
            object_begin2 = i;
            break;
        }
    }
    for (int i = object_begin1; i < P.size(); ++i) {
        if (P[i].y - P[i - 1].y > 10) {
            object_end2 = i;
            break;
        }
    }
    // std::cout << object_begin1 << " " << object_begin2 << " " << object_end1
    // << " " << object_end2 << std::endl;

    for (int i = 0; i < object_begin2; ++i) P_plane.push_back(P[i]);
    for (int i = object_end1; i < P.size(); ++i) P_plane.push_back(P[i]);
    for (int i = object_begin1; i < object_end2; ++i) P_object.push_back(P[i]);
}

/**
 * @brief Point2dto3d     二维像素坐标转三维图像坐标
 * @param n               图像序号
 * @param plane           平面方程系数
 * @param cameraMatrix    内参矩阵
 * @param distCoeffs      畸变系数
 * @param Pt2ds           二维点集
 * @param Pt3ds           三维点集
 */
void Calibration::Point2dto3d(int n, std::vector<double> plane,
                              cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
                              std::vector<cv::Point2d>& Pt2ds,
                              std::vector<cv::Point3d>& Pt3ds) {
    Q_UNUSED(distCoeffs)
    double A = plane[0], B = plane[1],
           C = plane[2];  // 平面方程为Ax + By + Cz + 1 = 0
    double u0 = cameraMatrix.at<double>(0, 2),
           v0 = cameraMatrix.at<double>(1, 2);  // 相机主点
    double fx = cameraMatrix.at<double>(0, 0),
           fy = cameraMatrix.at<double>(1, 1);  // 尺度因子

    std::ofstream fout("calibration//" + std::to_string(n) + "_3d.txt");
    for (int i = 0; i < Pt2ds.size(); ++i) {
        double u = Pt2ds[i].x, v = Pt2ds[i].y;
        /*
         * 图像坐标系转换到相机坐标系，得到的是归一化坐标值，也就是在z =
         * 1平面上的投影点， 所以求得的相机坐标系下的点其实是(x1, y1,
         * 1)，那么由原点通过点(x1, y1, 1)的直线方程为x/x1=y/y1=z，
         * 其中任一点x坐标满足x=x1*z，y坐标满足y=y1*z。
         * 带入平面方程Ax + By + Cz + 1 = 0可得：A*x1*z + B*y1*z + Cz + 1 = 0，
         * 即(A*x1 + B*y1 + C)*z + 1 = 0，所以z1 = -1 / (A*x1 + B*y1 + C)，
         * 认为向下为z轴负方向，则z1 = 1 / (A*x1 + B*y1 + C)。
         * 根据相似三角形，放缩比例为z1 / 1 =
         * z1，所以平面上的点的x和y坐标为x1*z1和y1*z1，推导完成。
         */
        double x1 = (double)((u - u0) / fx), y1 = (double)((v - v0) / fy);
        cv::Point3d pt;
        pt.z = (double)(1 / (A * x1 + B * y1 + C));
        pt.x = x1 * pt.z;
        pt.y = y1 * pt.z;
        Pt3ds.push_back(pt);
        fout << std::setprecision(16) << pt.x << " " << pt.y << " " << pt.z
             << std::endl;
    }
    fout.close();
}

/**
 * @brief PointtoPlaneEvaluation    点到面精度评价
 * @param Pt3ds                     三维点集
 * @param plane                     平面方程系数
 */
void Calibration::PointtoPlaneEvaluation(const std::vector<cv::Point3d>& Pt3ds,
                                         std::vector<double> plane) {
    double a = plane[0], b = plane[1], c = plane[2];
    std::vector<double> distance;

    double distance_mean = 0;
    for (int i = 0; i < Pt3ds.size(); ++i) {
        double dis = abs(a * Pt3ds[i].x + b * Pt3ds[i].y + c * Pt3ds[i].z - 1) /
                     sqrt(a * a + b * b + c * c);
        distance.push_back(dis);
        distance_mean += dis;
    }
    distance_mean /= Pt3ds.size();
    std::cout << "\n距离平均值：" << distance_mean << std::endl;

    double sigma = 0;
    for (int i = 0; i < Pt3ds.size(); ++i) {
        sigma += (distance[i] - distance_mean) * (distance[i] - distance_mean);
    }
    sigma /= Pt3ds.size();
    std::cout << "距离标准差：" << sqrt(sigma) << std::endl;
}

/**
 * @brief findPlane   最小二乘法拟合平面
 * @param pts         输入三维点集
 * @return            平面方程系数
 */
std::vector<double> Calibration::PlaneFittingRansac(
    std::vector<cv::Point3d>& pts) {
    //最小二乘法
    double A, B, C, D;
    std::vector<double> parameters;
    double meanX = 0, meanY = 0, meanZ = 0;
    double meanXX = 0, meanYY = 0, meanZZ = 0;
    double meanXY = 0, meanXZ = 0, meanYZ = 0;

    for (int i = 0; i < pts.size(); ++i) {
        meanX += pts[i].x;
        meanY += pts[i].y;
        meanZ += pts[i].z;

        meanXX += pts[i].x * pts[i].x;
        meanYY += pts[i].y * pts[i].y;
        meanZZ += pts[i].z * pts[i].z;

        meanXY += pts[i].x * pts[i].y;
        meanXZ += pts[i].x * pts[i].z;
        meanYZ += pts[i].y * pts[i].z;
    }
    meanX /= pts.size();
    meanY /= pts.size();
    meanZ /= pts.size();
    meanXX /= pts.size();
    meanYY /= pts.size();
    meanZZ /= pts.size();
    meanXY /= pts.size();
    meanXZ /= pts.size();
    meanYZ /= pts.size();

    /* eigenvector */
    Eigen::Matrix3d eMat;
    eMat(0, 0) = meanXX - meanX * meanX;
    eMat(0, 1) = meanXY - meanX * meanY;
    eMat(0, 2) = meanXZ - meanX * meanZ;
    eMat(1, 0) = meanXY - meanX * meanY;
    eMat(1, 1) = meanYY - meanY * meanY;
    eMat(1, 2) = meanYZ - meanY * meanZ;
    eMat(2, 0) = meanXZ - meanX * meanZ;
    eMat(2, 1) = meanYZ - meanY * meanZ;
    eMat(2, 2) = meanZZ - meanZ * meanZ;
    Eigen::EigenSolver<Eigen::Matrix3d> xjMat(eMat);
    Eigen::Matrix3d eValue =
        xjMat.pseudoEigenvalueMatrix();  // 得到协方差矩阵的特征值和特征向量
    Eigen::Matrix3d eVector = xjMat.pseudoEigenvectors();
    // 协方差矩阵的特征向量可以被看作是数据集中方差最小（或最不重要）的方向，而对应的特征值则表示了这个方向上的方差大小。
    // 因此，最小特征值对应的特征向量通常被认为是数据集中变化最小的方向，即是拟合平面的法向量。

    /* the eigenvector corresponding to the minimum eigenvalue */
    double v1 = eValue(0, 0);
    double v2 = eValue(1, 1);
    double v3 = eValue(2, 2);
    int minNumber = 0;
    if ((abs(v2) <= abs(v1)) && (abs(v2) <= abs(v3))) {
        minNumber = 1;
    }
    if ((abs(v3) <= abs(v1)) && (abs(v3) <= abs(v2))) {
        minNumber = 2;
    }
    A = eVector(0, minNumber);
    B = eVector(1, minNumber);
    C = eVector(2, minNumber);
    D = -(A * meanX + B * meanY + C * meanZ);

    /* result */
    if (C < 0) {
        A *= -1.0;
        B *= -1.0;
        C *= -1.0;
        D *= -1.0;
    }

    parameters.push_back(-A / D);
    parameters.push_back(-B / D);
    parameters.push_back(-C / D);
    return parameters;
}
