#include "steger.h"

Steger::Steger() {}

void Steger::getCenterLine(int num, cv::Mat& correct_image, cv::Mat& dst_image,
                           std::vector<cv::Point2f>& Points) {
    //    cv::Mat ori_img = cv::imread("image/2024-5-24_11-15-03.bmp", 0);
    //    dst_image = correct_image.clone();
    cv::Mat img1 = correct_image.clone();

    // 定义矩形区域
    //    cv::Rect rect(0, dst_image.rows / 3, dst_image.cols, dst_image.rows / 3);
    cv::Rect rect(0, 0, img1.cols, img1.rows);
    // 截取矩形区域
    cv::Mat img2 = img1(rect);
    cv::Mat img3 = img2.clone();
    dst_image = img2.clone();

    std::vector<cv::Point2f> pts;
    stegerMethod(img3, pts);

    cv::cvtColor(dst_image, dst_image, cv::COLOR_GRAY2RGB);
    Points.clear();
    for (size_t i = 0; i < pts.size(); ++i) {
        Points.push_back(cv::Point2f(pts[i].x, pts[i].y));

        cv::circle(dst_image, cv::Point(round(pts[i].x), round(pts[i].y)), 0, cv::Scalar(0, 0, 255),
                   -1);
    }
    cv::imwrite("./data/center/steger" + std::to_string(num) + ".bmp", dst_image);
}

bool Steger::stegerMethod(const cv::Mat& src, std::vector<cv::Point2f>& centers) {
    if (src.empty())  // src空，报错
    {
        return false;
    }
    centers.clear();  // 清空center内的内容

    cv::Mat img;  // 定义一个mat型变量
    if (src.type() ==
        CV_8UC3)  // CV_8UC3代表RGB的3个通道，就是说输入的彩色图像，一般图像格式是Unsigned 8bits
    {
        cvtColor(src, img, cv::COLOR_BGR2GRAY);  // 转换为灰度图像
    } else {
        img = src.clone();  // 类型不变，clone复制
    }

    img.convertTo(img, CV_32FC1);  // 将矩阵img转换为CV_32FC1类型的矩阵img，单精度浮点数格式。
    cv::GaussianBlur(img, img, cv::Size(0, 0), 6, 6);  // x,y方向的标准方差
    cv::imwrite("./data/center/steger_GaussianBlur.bmp", img);

    cv::Mat kernel_dx, kernel_dy;
    kernel_dx = (cv::Mat_<float>(1, 2) << -1,
                 1);  // 1行2列的 dx 模板  // https://www.cnblogs.com/xinxue/p/8494300.html
    kernel_dy = (cv::Mat_<float>(2, 1) << -1, 1);  // 2行1列的 dy 模板

    cv::Mat kernel_dxx, kernel_dyy, kernel_dxy;
    kernel_dxx = (cv::Mat_<float>(1, 3) << 1, -2, 1);      // 1行3列的 dxx 模板
    kernel_dyy = (cv::Mat_<float>(3, 1) << 1, -2, 1);      // 3行1列的 dyy 模板
    kernel_dxy = (cv::Mat_<float>(2, 2) << 1, -1, -1, 1);  // 2行2列的 dxy 模板

    // 一阶偏导
    cv::Mat dx_mat, dy_mat;
    filter2D(img, dx_mat, CV_32FC1, kernel_dx);
    filter2D(img, dy_mat, CV_32FC1, kernel_dy);

    // 二阶偏导
    cv::Mat dxx_mat, dyy_mat, dxy_mat;
    filter2D(img, dxx_mat, CV_32FC1, kernel_dxx);
    filter2D(img, dyy_mat, CV_32FC1, kernel_dyy);
    filter2D(img, dxy_mat, CV_32FC1, kernel_dxy);

    // Hessian矩阵
    //    double maxD = -1;
    int imgcol = img.cols;
    int imgrow = img.rows;

    for (int i = 0; i < imgcol; i++) {
        for (int j = 0; j < imgrow; j++) {
            if (src.at<uchar>(j, i) > 200)  // 像素值超过200
            {
                cv::Mat hessian(2, 2, CV_32FC1);
                hessian.at<float>(0, 0) = dxx_mat.at<float>(j, i);
                hessian.at<float>(0, 1) = dxy_mat.at<float>(j, i);
                hessian.at<float>(1, 0) = dxy_mat.at<float>(j, i);
                hessian.at<float>(1, 1) = dyy_mat.at<float>(j, i);

                cv::Mat eValue;
                cv::Mat eVectors;
                cv::eigen(hessian, eValue, eVectors);  // 计算特征值，或对称矩阵的特征值和特征向量

                double nx, ny;
                double fmaxD = 0;

                // 求特征值最大时对应的特征向量
                if (fabs(eValue.at<float>(0, 0)) >= fabs(eValue.at<float>(1, 0))) {
                    nx = eVectors.at<float>(0, 0);
                    ny = eVectors.at<float>(0, 1);
                    fmaxD = eValue.at<float>(0, 0);
                } else {
                    nx = eVectors.at<float>(1, 0);
                    ny = eVectors.at<float>(1, 1);
                    fmaxD = eValue.at<float>(1, 0);
                }

                double t =
                    -(nx * dx_mat.at<float>(j, i) + ny * dy_mat.at<float>(j, i)) /
                    (nx * nx * dxx_mat.at<float>(j, i) + 2 * nx * ny * dxy_mat.at<float>(j, i) +
                     ny * ny * dyy_mat.at<float>(j, i));
                if (fabs(t * nx) <= 0.5 && fabs(t * ny) <= 0.5) {
                    // 打印检测到的中心点到控制台
                    //                    cout << i + fabs(t * ny) << "\t" << j + fabs(t * nx) <<
                    //                    endl; qDebug() << i + t * nx << j + t * ny;
                    centers.push_back(cv::Point2f(i + t * nx, j + t * ny));
                }
            }
        }
    }
    return true;
}
