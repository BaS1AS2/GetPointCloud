#include "config.h"

#include <QThread>
#include <QTimer>

#include "myLable.h"
#include "ui_config.h"

config::config(QWidget *parent) : QWidget(parent), ui(new Ui::config) {
    ui->setupUi(this);
    // this->move(10,100);

    // ui->tableWidget->resizeColumnsToContents();

    on_pushButton_readData_clicked();
    on_pushButton_editSuc_clicked();

    QString wid, hig;
    wid = ui->imageSizeX->text();
    hig = ui->imageSizeY->text();
    imageSizeX = wid.toInt();
    imageSizeY = hig.toInt();

    showPixel = new QTimer(this);
    showPixel->setTimerType(Qt::PreciseTimer);
    connect(showPixel, SIGNAL(timeout()), this, SLOT(show_pixel()));

    writeConfigFile.open("calibration//calibration_result.txt");

    this->show();
}

config::~config() {
    delete ui;

    readFile.close();
    writeConfigFile.close();
}

// 相机内参
void config::setcameraMatrix_Fdx(double Fdx) { config_cameraMatrix_Fdx = Fdx; }
void config::setcameraMatrix_u0(double u0) { config_cameraMatrix_u0 = u0; }
void config::setcameraMatrix_Fdy(double Fdy) { config_cameraMatrix_Fdy = Fdy; }
void config::setcameraMatrix_v0(double v0) { config_cameraMatrix_v0 = v0; }

double config::getcameraMatrix_Fdx() { return config_cameraMatrix_Fdx; }
double config::getcameraMatrix_u0() { return config_cameraMatrix_u0; }
double config::getcameraMatrix_Fdy() { return config_cameraMatrix_Fdy; }
double config::getcameraMatrix_v0() { return config_cameraMatrix_v0; }

// 畸变系数
void config::setdistCoeffs_k1(double k1) { config_distCoeffs_k1 = k1; }
void config::setdistCoeffs_k2(double k2) { config_distCoeffs_k2 = k2; }
void config::setdistCoeffs_p1(double p1) { config_distCoeffs_p1 = p1; }
void config::setdistCoeffs_p2(double p2) { config_distCoeffs_p2 = p2; }

double config::getdistCoeffs_k1() { return config_distCoeffs_k1; }
double config::getdistCoeffs_k2() { return config_distCoeffs_k2; }
double config::getdistCoeffs_p1() { return config_distCoeffs_p1; }
double config::getdistCoeffs_p2() { return config_distCoeffs_p2; }

// 光平面方程
void config::setlaserPlane_a(double a) { config_laserPlane_a = a; }
void config::setlaserPlane_b(double b) { config_laserPlane_b = b; }
void config::setlaserPlane_c(double c) { config_laserPlane_c = c; }

double config::getlaserPlane_a() { return config_laserPlane_a; }
double config::getlaserPlane_b() { return config_laserPlane_b; }
double config::getlaserPlane_c() { return config_laserPlane_c; }

// 方向向量
void config::setChainDir_x(double dir_x) { config_chainDir_x = dir_x; }
void config::setChainDir_y(double dir_y) { config_chainDir_y = dir_y; }
void config::setChainDir_z(double dir_z) { config_chainDir_z = dir_z; }

double config::getChainDir_x() { return config_chainDir_x; }
double config::getChainDir_y() { return config_chainDir_y; }
double config::getChainDir_z() { return config_chainDir_z; }

// 输送链速度
void config::setChainSpeed(double speed) { config_chainSpeed = speed; }

double config::getChainSpeed() { return config_chainSpeed; }

// 用户坐标系相对于基座坐标系的直角坐标
void config::setTbuRx(double TbuRx) { config_Tbu_rx = TbuRx; }
void config::setTbuRy(double TbuRy) { config_Tbu_ry = TbuRy; }
void config::setTbuRz(double TbuRz) { config_Tbu_rz = TbuRz; }
void config::setTbuX(double TbuX) { config_Tbu_x = TbuX; }
void config::setTbuY(double TbuY) { config_Tbu_y = TbuY; }
void config::setTbuZ(double TbuZ) { config_Tbu_z = TbuZ; }

double config::getTbuRx() { return config_Tbu_rx; }
double config::getTbuRy() { return config_Tbu_ry; }
double config::getTbuRz() { return config_Tbu_rz; }
double config::getTbuX() { return config_Tbu_x; }
double config::getTbuY() { return config_Tbu_y; }
double config::getTbuZ() { return config_Tbu_z; }

// 标定处离喷涂处的距离
void config::setDiscalWithSpray(double DisCalWithSpray) {
    config_dis_calWithSpray = DisCalWithSpray;
}

double config::getDiscalWithSpray() { return config_dis_calWithSpray; }

// 手眼标定矩阵
void config::setEyeToHand(Eigen::Matrix4f eHMatrix) {
    config_EH_Matrix = eHMatrix;
}

Eigen::Matrix4f config::getEyeToHand() { return config_EH_Matrix; }

// 手眼标定标定板坐标系原点
void config::setORGXinCamera(double ORGX) { config_ORGX_inCamera = ORGX; }
void config::setORGYinCamera(double ORGY) { config_ORGY_inCamera = ORGY; }
void config::setORGZinCamera(double ORGZ) { config_ORGX_inCamera = ORGZ; }

double config::GetORGXinCamera() { return config_ORGX_inCamera; }
double config::GetORGYinCamera() { return config_ORGY_inCamera; }
double config::GetORGZinCamera() { return config_ORGZ_inCamera; }

// 采图处与标定处的距离
void config::setDisimgWithCal(double DisimgWithCal) {
    config_dis_imgWithCal = DisimgWithCal;
}

double config::getDisimgWithCal() { return config_dis_imgWithCal; }

// 参数传递
void config::on_pushButton_editSuc_clicked() {
    // 相机内参
    setcameraMatrix_Fdx(v_data[0]);

    setcameraMatrix_u0(v_data[1]);

    setcameraMatrix_Fdy(v_data[2]);

    setcameraMatrix_v0(v_data[3]);

    // 畸变系数
    setdistCoeffs_k1(v_data[4]);

    setdistCoeffs_k2(v_data[5]);

    setdistCoeffs_p1(v_data[6]);

    setdistCoeffs_p2(v_data[7]);

    // 光平面方程
    setlaserPlane_a(v_data[8]);

    setlaserPlane_b(v_data[9]);

    setlaserPlane_c(v_data[10]);

    // 方向向量
    setChainDir_x(v_data[11]);

    setChainDir_y(v_data[12]);

    setChainDir_z(v_data[13]);

    // 输送链速度
    setChainSpeed(v_data[14]);

    // 用户坐标系相对于基座坐标系的直角坐标
    setTbuRx(v_data[15]);

    setTbuRy(v_data[16]);

    setTbuRz(v_data[17]);

    setTbuX(v_data[18]);

    setTbuY(v_data[19]);

    setTbuZ(v_data[20]);

    // 标定处离喷涂处的距离
    setDiscalWithSpray(v_data[21]);

    // 手眼标定矩阵
    Eigen::Matrix4f eHMatrix;
    eHMatrix << v_data[22], v_data[25], v_data[28], v_data[31], v_data[23],
        v_data[26], v_data[29], v_data[32], v_data[24], v_data[27], v_data[30],
        v_data[33], 0, 0, 0, 1;

    setEyeToHand(eHMatrix);

    // 手眼标定标定板原点
    setORGXinCamera(v_data[34]);

    setORGYinCamera(v_data[35]);

    setORGZinCamera(v_data[36]);

    // 采图处与标定处的距离
    setDisimgWithCal(v_data[37]);
}

void config::on_pushButton_readData_clicked() {
    // 读入配置文件
    v_data.clear();
    readFile.open("./information/configFile.txt");  // 读入配置文件
    if (!readFile.is_open()) {
        std::cout << "read file failed!" << std::endl;
        return;
    }

    std::string line;
    while (std::getline(readFile, line)) {
        line_process(
            line);  // 把行首和行尾的多个空格, tab去掉，把注释文字也去掉
        if (line.empty()) continue;  // line为空则继续

        // 根据实际需求处理
        std::istringstream iss(line);
        double d_data;
        iss >> d_data;

        //        std::cout.precision(30);
        //        std::cout<<d_data<<std::endl;

        v_data.push_back(d_data);
    }

    // 设置表格
    if (v_data.size() <= 0) {
        return;
    }

    for (int i = 0; i < v_data.size(); i++) {
        QTableWidgetItem *item = new QTableWidgetItem();
        item->setText(QString::number(v_data[i]));
        ui->tableWidget->setItem(i, 2, item);
    }

    ui->tableWidget->resizeColumnsToContents();
}

void config::line_process(std::string &line, const std::string comment_str) {
    for (char &c : line)  // C++11以上版本的语法
    {
        // 制表符 tab，逗号，分号都当作有效的分隔符，统一转成空格
        // 为了避免错误，回车符和换行符也转为空格（否则无法处理空行）
        if (c == '\t' || c == ',' || c == ';' || c == '\r' || c == '\n')
            c = ' ';
    }

    line.erase(0, line.find_first_not_of(" "));  // 删除行首空格
    line.erase(line.find_last_not_of(" ") + 1);  // 删除行末空格
    // 调用的string& erase (size_t pos = 0, size_t len = npos);
    // len为默认参数

    // 查找注释符所在位置，如果不存在，则得到string::npos
    int n_comment_start = line.find_first_of(comment_str);
    if (n_comment_start != std::string::npos)  // 这一句必须的
        line.erase(n_comment_start);           //删除注释

    // 处理完毕。如果这一行只有空格，制表符 tab，注释，那么处理后line为空；
    // 如果行首有多个空格(或者空格和tab交错)，行尾为注释，那么处理后字符串line的
    // 行首多个空格(和tab)和行尾注释被删掉，只保留有意义的内容。
}

// 系统标定
void config::on_pushButton_calibration_clicked() {
    showPixel->stop();
    if (startEnd_2d.size() <= 0) {
        messenges = "please get start and end pixel first";
        ui->message->append(messenges);
        return;
    }

    // 相机标定
    messenges = "camera calibration start...";
    ui->message->append(messenges);
    std::vector<std::string> files;
    files.clear();
    std::vector<cv::Mat> tvecsMat, rvecsMat;
    tvecsMat.clear();
    rvecsMat.clear();
    cv::Mat cameraMatrix, distCoeffs;
    for (int i = 0; i <= 17; i++)
        files.push_back("calibration//" + std::to_string(i) + ".bmp");
    // files:文件名, cameraMatrix:内参矩阵, distCoeffs:畸变系数,
    // tvecsMat:平移矩阵, rvecsMat:旋转矩阵
    Calibration::CamraCalibration(files, cameraMatrix, distCoeffs, tvecsMat,
                                  rvecsMat);
    messenges = "camera calibration over";
    ui->message->append(messenges);
    writeConfigFile << "[**cameraMatrix**]" << std::endl
                    << cameraMatrix << std::endl
                    << std::endl;
    writeConfigFile << "[**distCoeffs**]" << std::endl
                    << distCoeffs << std::endl
                    << std::endl;

    // 光平面标定
    writeConfigFile << "/**Laser plane calibration**/" << std::endl;
    std::vector<cv::Point3d> Pt3ds_n;
    Pt3ds_n.clear();
    std::vector<double> plane;
    plane.clear();
    plane.resize(3);
    std::string filename;
    cv::Mat image, image_corrected;
    cv::Point2d pt;
    double x, y, z;
    int start, end;
    ifstream infile;
    //    for(int i = 0; i < startEnd_2d.size(); i++)
    //    {
    //        std::cout << "startEnd_2d" << startEnd_2d[i] << std::endl;
    //    }

    //    for (int i = 0; i < 2; i++)
    //    {
    //        // 保存前四张图像的旋转矩阵
    //        cv::Mat rotation_matrix = cv::Mat(3, 3, CV_32FC1,
    //        cv::Scalar::all(0));
    //        //
    //        将旋转向量rvecsMat[i]转换为旋转矩阵rotation_matrix，旋转方向由旋转向量的方向确定，旋转角度即旋转向量的模
    //        cv::Rodrigues(rvecsMat[i], rotation_matrix);
    //        // 将旋转和平移矩阵写入标定文档
    //        writeConfigFile << "tvecsMat " << i << std::endl << tvecsMat[i] <<
    //        std::endl; writeConfigFile << "rotation_matrix " << i << std::endl
    //        << rotation_matrix << std::endl; writeConfigFile << std::endl;
    //    }
    for (int i = 10; i <= 13; i++) {
        // 10~13每幅图像的旋转矩阵
        cv::Mat rotation_matrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));

        // 将旋转向量rvecsMat[i]转换为旋转矩阵rotation_matrix，旋转方向由旋转向量的方向确定，旋转角度即旋转向量的模
        cv::Rodrigues(rvecsMat[i], rotation_matrix);

        // 将旋转和平移矩阵写入标定文档
        writeConfigFile << "tvecsMat " << i << std::endl
                        << tvecsMat[i] << std::endl;
        writeConfigFile << "rotation_matrix " << i << std::endl
                        << rotation_matrix << std::endl;
        writeConfigFile << std::endl;

        // 生成每张标定板所在平面方程
        /* 相关公式推导：
         * 原本的平面为z=0，它的法向量为[0, 0,
         * 1]^T，法向量乘以旋转矩阵即可得到变换后的平面的法向量，为[R13, R23,
         * R33] 那么假设平面方程为R13x + R23y + R33z + D =
         * 0，此时只有D是未知数了。取原本平面上的任意一点为[x, y,
         * 0]，对它进行旋转平移变换后得到点： [R11x+R12y+t11, R21x+R22y+t21,
         * R31x+R32y+t31]，将这个点带入平面方程R13x + R23y + R33z + D =
         * 0，得到： R13(R11x+R12y+t11) + R23(R21x+R22y+t21) +
         * R33(R31x+R32y+t31) + D =
         * 0，该平面方程对变换后的任意点都成立，那么对点(0, 0)也成立， 令 x = y
         * = 0，即可得到 D = -(R13*t11 + R23*t21 +
         * R33*t31)。最后再对平面方程做归一化处理，即R13/D，R23/D，R33/D，完成方程求解。
         */
        std::vector<cv::Point2d> Pts2d;  // 提取到的所有激光条纹中心线点坐标
        std::vector<cv::Point2d>
            Pts2d_n;  // 在所点击的起始以及终止点之间的条纹中心线点坐标
        float d1 =
            -(tvecsMat[i].at<double>(0, 0) * rotation_matrix.at<double>(0, 2) +
              tvecsMat[i].at<double>(0, 1) * rotation_matrix.at<double>(1, 2) +
              tvecsMat[i].at<double>(0, 2) * rotation_matrix.at<double>(2, 2));
        plane[0] = rotation_matrix.at<double>(0, 2) /
                   d1;  // 得到标定板的平面方程 Ax + By + Cz + 1 = 0
        plane[1] = rotation_matrix.at<double>(1, 2) /
                   d1;  // plane[0]、plane[1]、plane[2]分别为A、B、C
        plane[2] = rotation_matrix.at<double>(2, 2) / d1;

        // 生成标定板上激光条纹的三维坐标
        filename = "calibration//J" + std::to_string(i) + ".bmp";
        Calibration::GetImage(filename, image);
        Calibration::Correction(image, image_corrected, cameraMatrix,
                                distCoeffs);
        Calibration::CenterLine(i, image_corrected, Pts2d);

        infile.open("calibration//" + std::to_string(i) + "_2d.txt");
        start = startEnd_2d[2 * (i - 10)].x;  // 获取点击起点终点时候的坐标
        end = startEnd_2d[2 * (i - 10) + 1].x;
        while (infile >> x >>
               y) {  // 从二维坐标点文件中依次取出每个中心线上的点
            if (x > start &&
                x < end) {  // 点如果在所选择的标定板起始终止点之间，就保留
                pt.x = x;
                pt.y = y;
                Pts2d_n.push_back(pt);
            }
        }
        infile.close();
        Calibration::Point2dto3d(
            i, plane, cameraMatrix, distCoeffs, Pts2d_n,
            Pt3ds_n);  // 计算标定板上的激光条纹点的空间三维坐标
    }
    std::vector<double> light_plane;
    light_plane.clear();
    light_plane = Calibration::PlaneFittingRansac(
        Pt3ds_n);  // 将第10~13张图片中线激光转化的点云拟合成一个平面，这个平面就是激光平面。
    writeConfigFile << "[**light_plane**]" << light_plane[0] << ","
                    << light_plane[1] << "," << light_plane[2] << std::endl
                    << std::endl;
    Pt3ds_n.clear();
    messenges = "plane calibration over";
    ui->message->append(messenges);
    startEnd_2d.clear();

    // 方向向量标定
    writeConfigFile << "/**chain direction calibration**/" << std::endl;
    cv::Point3f dir_pt;
    std::vector<cv::Point3f> pts;
    pts.clear();
    for (int i = 14; i <= 16; i++) {
        writeConfigFile << "tvecsMat " << i << std::endl
                        << tvecsMat[i] << std::endl;
        writeConfigFile << std::endl;
        dir_pt.x = tvecsMat[i].at<double>(0, 0);
        dir_pt.y = tvecsMat[i].at<double>(0, 1);
        dir_pt.z = tvecsMat[i].at<double>(0, 2);
        pts.push_back(dir_pt);
    }
    double delta_x = pts[1].x - pts[0].x;
    double delta_y = pts[1].y - pts[0].y;
    double delta_z = pts[1].z - pts[0].z;

    writeConfigFile
        << "[**chain direction**]" << std::fixed << std::setprecision(6)
        << delta_x * sqrt(1 / (delta_x * delta_x + delta_y * delta_y +
                               delta_z * delta_z))
        << ","
        << delta_y * sqrt(1 / (delta_x * delta_x + delta_y * delta_y +
                               delta_z * delta_z))
        << ","
        << delta_z * sqrt(1 / (delta_x * delta_x + delta_y * delta_y +
                               delta_z * delta_z))
        << std::endl
        << std::endl;
    messenges = "direction calibration over";
    ui->message->append(messenges);

    // 手眼标定Tbc=Tbu*Tcw_
    writeConfigFile << "/**eye to hand**/" << std::endl;
    cv::Mat rotation_matrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
    cv::Rodrigues(rvecsMat[17], rotation_matrix);
    writeConfigFile << "tvecsMat " << 17 << std::endl
                    << tvecsMat[17] << std::endl;
    writeConfigFile << "rotation_matrix " << 17 << std::endl
                    << rotation_matrix << std::endl;
    writeConfigFile << std::endl;
    /* 获得Tbu */
    double bu_rx = config_Tbu_rx * M_PI / 180,
           bu_ry = config_Tbu_ry * M_PI / 180,
           bu_rz = config_Tbu_rz * M_PI / 180, bu_x = config_Tbu_x,
           bu_y = config_Tbu_y, bu_z = config_Tbu_z, nx, ny, nz, ox, oy, oz, ax,
           ay, az;
    nx = cos(bu_ry) * cos(bu_rz);
    ny = cos(bu_ry) * sin(bu_rz);
    nz = -sin(bu_ry);
    ox = sin(bu_rx) * sin(bu_ry) * cos(bu_rz) - cos(bu_rx) * sin(bu_rz);
    oy = sin(bu_rx) * sin(bu_ry) * sin(bu_rz) + cos(bu_rx) * cos(bu_rz);
    oz = sin(bu_rx) * cos(bu_ry);
    ax = cos(bu_rx) * sin(bu_ry) * cos(bu_rz) + sin(bu_rx) * sin(bu_rz);
    ay = cos(bu_rx) * sin(bu_ry) * sin(bu_rz) - sin(bu_rx) * cos(bu_rz);
    az = cos(bu_rx) * cos(bu_ry);
    Eigen::Matrix4f Tbu = Eigen::Matrix4f::Identity();
    ;
    Tbu << nx, ox, ax, bu_x, ny, oy, ay, bu_y, nz, oz, az, bu_z, 0, 0, 0, 1;
    writeConfigFile << "T_bu " << std::endl << Tbu << std::endl << std::endl;
    /* 获得Tcw_ */
    double dx = config_chainDir_x, dy = config_chainDir_y,
           dz = config_chainDir_z;  // 方向向量
    float move = config_dis_calWithSpray;

    cv::Mat rotation_matrix_ofCali_inCamera =
        cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
    Rodrigues(rvecsMat[17], rotation_matrix_ofCali_inCamera);
    nx = rotation_matrix_ofCali_inCamera.at<double>(0, 0);
    ny = rotation_matrix_ofCali_inCamera.at<double>(1, 0);
    nz = rotation_matrix_ofCali_inCamera.at<double>(2, 0);
    ox = rotation_matrix_ofCali_inCamera.at<double>(0, 1);
    oy = rotation_matrix_ofCali_inCamera.at<double>(1, 1);
    oz = rotation_matrix_ofCali_inCamera.at<double>(2, 1);
    ax = rotation_matrix_ofCali_inCamera.at<double>(0, 2);
    ay = rotation_matrix_ofCali_inCamera.at<double>(1, 2);
    az = rotation_matrix_ofCali_inCamera.at<double>(2, 2);
    x = tvecsMat[17].at<double>(0, 0);
    y = tvecsMat[17].at<double>(0, 1);
    z = tvecsMat[17].at<double>(0, 2);
    x = x + move * dx;
    y = y + move * dy;
    z = z + move * dz;
    Eigen::Matrix4f Tcw_ = Eigen::Matrix4f::Identity();
    Tcw_ << nx, ox, ax, x, ny, oy, ay, y, nz, oz, az, z, 0, 0, 0, 1;
    writeConfigFile << "Tcw_ " << std::endl << Tcw_ << std::endl << std::endl;
    writeConfigFile << "Tcw_.inverse " << std::endl
                    << Tcw_.inverse() << std::endl
                    << std::endl;

    Eigen::Matrix4f Tbc = Eigen::Matrix4f::Identity();
    ;
    Tbc = Tbu * Tcw_.inverse();
    writeConfigFile << "[**Tbc**]" << std::endl << Tbc << std::endl;
    messenges = "eye to hand calibration over";
    ui->message->append(messenges);
}

void config::on_pushButton_getStart_clicked() {
    cv::Point2f start;
    start.x = x_in_Img;
    start.y = y_in_Img;
    startEnd_2d.push_back(start);

    messenges = QString::number(start.x) + " , " + QString::number(start.y);
    ui->message->append(messenges);
}

void config::on_pushButton_getEnd_clicked() {
    cv::Point2f end;
    end.x = x_in_Img;
    end.y = y_in_Img;
    startEnd_2d.push_back(end);

    messenges = QString::number(end.x) + " , " + QString::number(end.y);
    ui->message->append(messenges);
}

void config::on_pushButton_nextLaser_clicked() {
    showPixel->start(100);
    static int i = 10;
    src = cv::imread("calibration//J" + std::to_string(i) + ".bmp");
    QImage Qtemp = QImage((const unsigned char *)(src.data), src.cols, src.rows,
                          src.step, QImage::Format_RGB888);
    QImage newQtemp = Qtemp.scaled(ui->label->size());
    ui->label->setPixmap(QPixmap::fromImage(newQtemp));
    ui->label->show();

    messenges = "give picture " + QString::number(i) + " start and end:";
    ui->message->append(messenges);

    i++;
    if (i > 14) {
        i = 0;
        messenges = "Laser picture is end";
        ui->message->append(messenges);
        QMessageBox *box =
            new QMessageBox(QMessageBox::Warning, u8"warning",
                            u8"Laser picture is over", QMessageBox::Ok);
        if (NULL != box->button(QMessageBox::Ok))
            box->button(QMessageBox::Ok)->setText(u8"yes");
        QTimer::singleShot(3000, box, SLOT(accept()));
        box->exec();
        return;
    }
}

void config::show_pixel() {
    cv::Mat src_image = src.clone();
    cv::circle(src_image, cv::Point(x_in_Img, y_in_Img), 5,
               cv::Scalar(255, 0, 0), -1);
    ui->pixel_x->setNum(x_in_Img);
    ui->pixel_y->setNum(y_in_Img);
    QImage Qtemp =
        QImage((const unsigned char *)(src_image.data), src_image.cols,
               src_image.rows, src_image.step, QImage::Format_RGB888);
    QImage newQtemp = Qtemp.scaled(ui->label->size());
    ui->label->setPixmap(QPixmap::fromImage(newQtemp));
    ui->label->show();
}
