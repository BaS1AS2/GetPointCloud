#ifndef CONFIG_H
#define CONFIG_H
#pragma execution_character_set("utf-8")

// #include <algorithm>
// #include <cassert>
// #include <iomanip>
// #include <sstream>

#include <QWidget>

#include "camera_calibration.h"

namespace Ui {
class config;
}

class config : public QWidget {
    Q_OBJECT

public:
    explicit config(QWidget *parent = nullptr);
    ~config();

    /**
     * @brief messenges调试信息
     */
    QString messenges = 0;

    int imageSizeX;

    int imageSizeY;

    // 相机内参
    void setcameraMatrix_Fdx(double Fdx);
    void setcameraMatrix_u0(double u0);
    void setcameraMatrix_Fdy(double Fdy);
    void setcameraMatrix_v0(double v0);

    double getcameraMatrix_Fdx();
    double getcameraMatrix_u0();
    double getcameraMatrix_Fdy();
    double getcameraMatrix_v0();

    // 畸变系数
    void setdistCoeffs_k1(double k1);
    void setdistCoeffs_k2(double k2);
    void setdistCoeffs_p1(double p1);
    void setdistCoeffs_p2(double p2);

    double getdistCoeffs_k1();
    double getdistCoeffs_k2();
    double getdistCoeffs_p1();
    double getdistCoeffs_p2();

    // 光平面方程
    void setlaserPlane_a(double a);
    void setlaserPlane_b(double b);
    void setlaserPlane_c(double c);

    double getlaserPlane_a();
    double getlaserPlane_b();
    double getlaserPlane_c();

    // 方向向量
    void setChainDir_x(double dir_x);
    void setChainDir_y(double dir_y);
    void setChainDir_z(double dir_z);

    double getChainDir_x();
    double getChainDir_y();
    double getChainDir_z();

    // 输送链速度
    void setChainSpeed(double speed);

    double getChainSpeed();

    // 用户坐标系相对于基座坐标系的直角坐标
    void setTbuRx(double TbuRx);
    void setTbuRy(double TbuRy);
    void setTbuRz(double TbuRz);
    void setTbuX(double TbuX);
    void setTbuY(double TbuY);
    void setTbuZ(double TbuZ);

    double getTbuRx();
    double getTbuRy();
    double getTbuRz();
    double getTbuX();
    double getTbuY();
    double getTbuZ();

    // 标定处离喷涂处的距离
    void setDiscalWithSpray(double DisImgWithSpray);

    double getDiscalWithSpray();

    // 手眼标定矩阵
    void setEyeToHand(Eigen::Matrix4f eHMatrix);

    Eigen::Matrix4f getEyeToHand();

    // 手眼标定标定板坐标系原点
    void setORGXinCamera(double ORGX);
    void setORGYinCamera(double ORGY);
    void setORGZinCamera(double ORGZ);

    double GetORGXinCamera();
    double GetORGYinCamera();
    double GetORGZinCamera();

    // 采图处与标定处的距离
    void setDisimgWithCal(double DisimgWithCal);

    double getDisimgWithCal();

private slots:

    void on_pushButton_editSuc_clicked();

    void on_pushButton_readData_clicked();

    void on_pushButton_calibration_clicked();

    void on_pushButton_getStart_clicked();

    void on_pushButton_getEnd_clicked();

    void on_pushButton_nextLaser_clicked();

    void show_pixel();

private:
    Ui::config *ui;

    std::ifstream readFile;  // 读配置文件

    std::ofstream writeConfigFile;  // 保存标定结果的文件

    std::vector<double> v_data;

    cv::Mat src;

    // 相机内参
    double config_cameraMatrix_Fdx;
    double config_cameraMatrix_u0;
    double config_cameraMatrix_Fdy;
    double config_cameraMatrix_v0;

    // 畸变系数
    double config_distCoeffs_k1;
    double config_distCoeffs_k2;
    double config_distCoeffs_p1;
    double config_distCoeffs_p2;

    // 光平面方程
    double config_laserPlane_a;
    double config_laserPlane_b;
    double config_laserPlane_c;

    // 方向向量
    double config_chainDir_x;
    double config_chainDir_y;
    double config_chainDir_z;

    // 输送链速度
    double config_chainSpeed;

    // 用户坐标系相对于基座坐标系的直角坐标
    double config_Tbu_rx;
    double config_Tbu_ry;
    double config_Tbu_rz;
    double config_Tbu_x;
    double config_Tbu_y;
    double config_Tbu_z;

    // 采图处离喷涂处的距离
    float config_dis_calWithSpray;

    // 手眼标定矩阵
    Eigen::Matrix4f config_EH_Matrix;

    // 标定板坐标系原点
    double config_ORGX_inCamera;
    double config_ORGY_inCamera;
    double config_ORGZ_inCamera;

    // 采图处与标定处的距离
    double config_dis_imgWithCal;

    // 标定二维图存储起点和终点
    std::vector<cv::Point2f> startEnd_2d;

    // 二维像素坐标显示定时器
    QTimer *showPixel = new QTimer(this);

    /**
     * @brief line_process处理注释，空格，和空行的函数
     * @param line表示一行文本内容
     * @param comment_str表示注释前导字符串，默认设置为#，也可以用//或者%
     */
    void line_process(std::string &line, const std::string comment_str = "#");
};

#endif  // CONFIG_H
