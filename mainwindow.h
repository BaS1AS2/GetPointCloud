#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QVTKWidget.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <pylon/PylonIncludes.h>
#include <pylon/gige/BaslerGigECamera.h>
#include <vtkAutoInit.h>
#include <vtkRenderWindow.h>

#include <QMainWindow>
#include <QTimer>

#include "config/config.h"
#include "image_processing/image_processing.h"
#include "image_processing/normalcentroid.h"
#include "image_processing/steger.h"
#include "lib/zauxdll2.h"
#include "lib/zmotion.h"
#include "qdatetime.h"

VTK_MODULE_INIT(vtkRenderingOpenGL2)
VTK_MODULE_INIT(vtkInteractionStyle)
VTK_MODULE_INIT(vtkRenderingFreeType)

using pcl::visualization::PointCloudColorHandlerCustom;
using pcl::visualization::PointCloudColorHandlerGenericField;

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    // 单帧点云
    void image_capture();

    void on_open_camera_clicked();

    void on_close_camera_clicked();

    void on_get_image_clicked();

    void on_imageShow_clicked();

    void on_get_centerline_clicked();

    void on_get_ptcloud_clicked();

    // 旋转拼接
    void read_angle();

    void capture_rotation();

    bool move_controll_connect();

    void on_get_calibration_clicked();

    void on_capture_rotation_clicked();

    void on_splicing_rotation_clicked();

    // 线性拼接
    void read_move();

    void capture_linear();

    void on_capture_linear_clicked();

    void on_splicing_linear_clicked();

    void on_btn_calibration_clicked();

private:
    Ui::MainWindow *ui;

    // 单帧点云
    cv::Mat src;                  // 采图图像
    cv::Mat correct;              // 矫正图像
    cv::Mat dst;                  // 结果图像
    std::vector<cv::Point2f> Pt;  // 激光条纹中心点校正后的像素坐标
    int grabbed_images = 0;       // 采集到的图像数
    QTimer *photo_capture_timer = new QTimer(this);  // 采集图像定时器
    QString messenges = 0;                           // 调试信息
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloud;     // 单帧点云轮廓的三维坐标
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;  // 点云显示

    Pylon::DeviceInfoList_t device;                // pylon相机设备
    Pylon::CBaslerUniversalInstantCamera camera;   // pylon相机对象
    Pylon::CImageFormatConverter formatConverter;  // pylon格式对象
    Pylon::CPylonImage pylonImage;                 // PylonImage对象
    Pylon::CGrabResultPtr ptrGrabResult;           // 抓取结果数据指针
    bool camera_flag = true;                       // 相机是否开启的标记

    // 旋转拼接
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;    // 采集到的点云
    float angle;                                  // 角度
    QTimer *read_angle_timer = new QTimer(this);  // 采集转角定时器
    char *ip_addr;                                // 运动控制器ip地址
    ZMC_HANDLE handle = NULL;                     // 运动控制器连接句柄
    bool move_flag = false;                       // 运动控制器是否开启的标记

    // 线性拼接
    float distance = 0;                          // 距离
    float startPosition = 0;                     // 初始位置
    QTimer *read_move_timer = new QTimer(this);  // 采集位移定时器
};

#endif  // MAINWINDOW_H
