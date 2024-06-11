#include "mainwindow.h"

#include "move_control/move_control.h"
#include "ui_mainwindow.h"

#pragma comment(lib, "D:/BaiduSyncdisk/Pot/GetPointCloud/lib/zauxdll.lib")
#pragma comment(lib, "D:/BaiduSyncdisk/Pot/GetPointCloud/lib/zmotion.lib")

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);

    ptcloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));

    connect(ui->exposure, &QLineEdit::textChanged, this, &MainWindow::on_open_camera_clicked);

    // 打开页面时，直接读取标定参数以及路径等参数
    ui->get_calibration->click();

    const char *addr = "192.168.1.11";
    ip_addr = const_cast<char *>(addr);
}

MainWindow::~MainWindow() { delete ui; }

/**
 * @brief read_angles 角度
 */
std::vector<float> read_angles;

/**
 * @brief read_moves 位置
 */
std::vector<float> read_moves;

float dirx, diry, dirz;

// 单帧点云
/**
 * @brief image_capture 采图
 */
void MainWindow::image_capture() {
    //    double time_start = static_cast<double>(cv::getTickCount());
    cv::Mat opencvImage;

    // 等待接收和恢复图像，超时时间设置为5000ms.
    camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
    if (ptrGrabResult->GrabSucceeded()) {
        //        std::cout << QDateTime::currentDateTime().toString("hh:mm:ss.zz").toStdString() <<
        //        std::endl;

        formatConverter.Convert(pylonImage, ptrGrabResult);  // 将抓取的缓冲数据转化成pylonImage
        opencvImage = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC1,
                              (uint8_t *)pylonImage.GetBuffer());  // 将pylonimage转成OpenCV image.

        cv::imwrite("./data/image/" + std::to_string(grabbed_images) + ".bmp", opencvImage);
    }

    ui->image_window->clear();
    QImage qimage = QImage((const uchar *)opencvImage.data, opencvImage.cols, opencvImage.rows,
                           QImage::Format_Grayscale8)
                        .rgbSwapped();
    ui->image_window->setPixmap(QPixmap::fromImage(qimage));
    ui->image_window->show();

    ++grabbed_images;

    // 判断采集数目是否完成
    QString imageNum;
    imageNum = ui->image_num->text();
    if (grabbed_images > imageNum.toDouble()) {
        std::cout << "capture finished!" << std::endl;
        photo_capture_timer->stop();
        ui->image_window->clear();
        ui->open_camera->setEnabled(true);
        ui->close_camera->setEnabled(true);
    }
}

void MainWindow::on_open_camera_clicked() {
    Pylon::PylonInitialize();  // 初始化Pylon对象

    // 判断相机连接情况
    if (!Pylon::CTlFactory::GetInstance().EnumerateDevices(device)) {
        QMessageBox *box = new QMessageBox(QMessageBox::Warning, "warning!",
                                           "Check camera connection!", QMessageBox::Ok);
        if (NULL != box->button(QMessageBox::Ok)) {
            box->button(QMessageBox::Ok)->setText("yes");
        }
        QTimer::singleShot(3000, box, SLOT(accept()));
        box->exec();
        return;
    }

    // 关联检测到的第一台相机设备
    camera.Attach(Pylon::CTlFactory::GetInstance().CreateDevice(device[0]));

    // 判断相机是否被其他程序占用
    try {
        camera.Open();  // 打开相机
    } catch (...) {
        QMessageBox *box = new QMessageBox(QMessageBox::Warning, "warning!",
                                           "The camera is already in use!", QMessageBox::Ok);
        if (NULL != box->button(QMessageBox::Ok)) {
            box->button(QMessageBox::Ok)->setText("yes");
        }

        QTimer::singleShot(3000, box, SLOT(accept()));
        box->exec();
        return;
    }

    camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);  // 相机默认设置连续抓取模式
    formatConverter.OutputPixelFormat = Pylon::PixelType_Mono8;  // 确定输出像素格式

    // 设置相机曝光时间
    GenApi::INodeMap &cameraNodeMap = camera.GetNodeMap();
    const GenApi::CFloatPtr exposureTime = cameraNodeMap.GetNode("ExposureTimeAbs");

    QString exposure;
    exposure = ui->exposure->text();
    exposureTime->SetValue(exposure.toDouble());

    camera_flag = true;

    // 循环采图
    while (camera_flag) {
        std::cout << QDateTime::currentDateTime().toString("hh:mm:ss.zz").toStdString()
                  << std::endl;

        // 等待接收和恢复图像，超时时间设置为5000ms.
        camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
        if (ptrGrabResult->GrabSucceeded()) {
            formatConverter.Convert(pylonImage, ptrGrabResult);  // 将抓取的缓冲数据转化成pylonImage
            // 将pylonimage转成OpenCV image.
            cv::Mat opencvImage = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(),
                                          CV_8UC1, (uint8_t *)pylonImage.GetBuffer());

            ui->image_window->clear();
            ui->image_window->setScaledContents(true);  // 自适应大小
            QImage qimage = QImage((const uchar *)opencvImage.data, opencvImage.cols,
                                   opencvImage.rows, QImage::Format_Grayscale8)
                                .rgbSwapped();
            ui->image_window->setPixmap(
                QPixmap::fromImage(qimage).scaled(ui->image_window->size(), Qt::KeepAspectRatio));
            ui->image_window->show();
        }
        cv::waitKey(50);  // 防止采图卡顿
    }
}

void MainWindow::on_close_camera_clicked() {
    camera.Close();
    camera.DetachDevice();
    Pylon::PylonTerminate();
    camera_flag = false;
    ui->image_window->clear();
}

void MainWindow::on_get_image_clicked() {
    ui->open_camera->setEnabled(false);
    ui->close_camera->setEnabled(false);

    Pylon::PylonInitialize();  // 初始化Pylon对象

    // 判断相机连接情况
    if (!Pylon::CTlFactory::GetInstance().EnumerateDevices(device)) {
        QMessageBox *box = new QMessageBox(QMessageBox::Warning, "warning!",
                                           "Check camera connection!", QMessageBox::Ok);
        if (NULL != box->button(QMessageBox::Ok)) {
            box->button(QMessageBox::Ok)->setText("yes");
        }
        QTimer::singleShot(3000, box, SLOT(accept()));
        box->exec();
        return;
    }

    camera.Attach(
        Pylon::CTlFactory::GetInstance().CreateDevice(device[0]));  // 关联检测到的第一台相机设备

    // 判断相机是否被其他程序占用
    try {
        camera.Open();  // 打开相机
    } catch (...) {
        QMessageBox *box = new QMessageBox(QMessageBox::Warning, "warning!",
                                           "The camera is already in use!", QMessageBox::Ok);
        if (NULL != box->button(QMessageBox::Ok)) {
            box->button(QMessageBox::Ok)->setText("yes");
        }

        QTimer::singleShot(3000, box, SLOT(accept()));
        box->exec();
        return;
    }

    camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);  // 相机默认设置连续抓取模式
    formatConverter.OutputPixelFormat = Pylon::PixelType_Mono8;  // 确定输出像素格式

    // 设置相机曝光时间
    GenApi::INodeMap &cameraNodeMap = camera.GetNodeMap();
    const GenApi::CFloatPtr exposureTime = cameraNodeMap.GetNode("ExposureTimeAbs");
    QString exposure;
    exposure = ui->exposure->text();
    exposureTime->SetValue(exposure.toDouble());

    // 采图定时器初始化
    photo_capture_timer = new QTimer(this);
    photo_capture_timer->setTimerType(Qt::PreciseTimer);
    connect(photo_capture_timer, SIGNAL(timeout()), this, SLOT(image_capture()));  // 定时采图
    QString capture;
    capture = ui->capture_timer->text();
    photo_capture_timer->start(capture.toInt());
}

void MainWindow::on_imageShow_clicked() {
    QString i;
    i = ui->image_i->text();
    // 获取图像
    ImageProcessing::GetImage(i.toInt(), src);
    ImageProcessing::Correction(src, correct);
    //    correct = src;

    // 调试信息
    if (correct.rows * correct.cols) {
        messenges = "show " + QString::number(i.toInt()) + " image successfully.";
        ui->message->append(messenges);
    } else {
        messenges = "correction error!";
        ui->message->append(messenges);
    }

    // 显示图片
    // cv::imshow("src",src);
    ui->image_window->clear();

    ui->image_window->setScaledContents(true);  // 自适应大小
    QImage image =
        QImage((const uchar *)correct.data, correct.cols, correct.rows, QImage::Format_Indexed8)
            .rgbSwapped();
    ui->image_window->setPixmap(
        QPixmap::fromImage(image).scaled(ui->image_window->size(), Qt::KeepAspectRatio));
    ui->image_window->show();
}

void MainWindow::on_get_centerline_clicked() {
    QString i;
    i = ui->image_i->text();
    int num = i.toInt();

    // 提取中心线
    try {
        // 创建一个QElapsedTimer对象
        QElapsedTimer timer;
        // 开始计时
        timer.start();

        // 灰度重心法（原本的）
        //        ImageProcessing::CenterLineHorizontal(num, correct, dst, Pt);  // 提取中心线

        // 法向质心法
        NormalCentroid n;
        n.getCenterLine(num, correct, dst, Pt);

        // steger算法
        //        Steger s;
        //        s.getCenterLine(num, correct, dst, Pt);

        // 获取运行时间（纳秒, 微秒/1000, 毫秒/1000/1000）
        qint64 elapsedMicroseconds = timer.nsecsElapsed() / 1000 / 1000;
        qDebug() << "Center line:" << elapsedMicroseconds << "ms";
    } catch (...) {
        std::cout << "image processing error!" << std::endl << "ms";
    }

    // 调试信息
    if (dst.rows * dst.cols) {
        messenges = "get center line of " + QString::number(i.toInt()) + " image successfully.";
        ui->message->append(messenges);
    } else {
        messenges = "get center line error!";
        ui->message->append(messenges);
    }

    // 显示中心线
    ui->center_window->clear();
    ui->center_window->setScaledContents(true);  // 自适应大小
    QImage image =
        QImage((const uchar *)dst.data, dst.cols, dst.rows, QImage::Format_RGB888).rgbSwapped();
    ui->center_window->setPixmap(QPixmap::fromImage(image).scaled(
        ui->image_window->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    ui->center_window->show();
}

void MainWindow::on_get_ptcloud_clicked() {
    QString i;
    i = ui->image_i->text();

    // 计算三维轮廓坐标
    ImageProcessing::Point2dto3d(i.toInt(), Pt, ptcloud);

    // 调试信息
    if (ptcloud->size()) {
        messenges = "get pt cloud " + QString::number(ptcloud->size()) + " of " +
                    QString::number(i.toInt()) + " image successfully.";
        ui->message->append(messenges);
    } else {
        messenges = "get pt cloud error!";
        ui->message->append(messenges);
    }

    // =========================== 显示点云 ===========================
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();

    for (int i = 0; i < ptcloud->size(); i++) {
        vtkIdType pid[1];
        pid[0] = points->InsertNextPoint(ptcloud->at(i).x, ptcloud->at(i).y, ptcloud->at(i).z);
        vertices->InsertNextCell(1, pid);
    }

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetVerts(vertices);

    // 映射
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    // 演员
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(1.0, 1.0, 1.0);
    actor->GetProperty()->SetPointSize(1);

    vtkRenderer *renderer = vtkRenderer::New();
    renderer->AddActor(actor);
    renderer->ResetCamera();
    renderer->SetBackground(0, 0, 0);

    ui->qvtkWidget->GetRenderWindow()->AddRenderer(renderer);
    ui->qvtkWidget->GetRenderWindow()->Render();
}

// 旋转拼接
/**
 * @brief read_turnTable_angle 读转角
 */
void MainWindow::read_angle() {
    QString axis;
    axis = ui->axis->text();
    //    int ret = ZAux_Direct_GetDpos(handle,axis.toInt(), &angle);
    // ZAux_Direct_GetTable(handle, 990, 1, &angle);
    //    std::cout << QDateTime::currentDateTime().toString("hh:mm:ss.zz").toStdString() <<
    //    std::endl; std::cout<<"angle "<<angle<<std::endl;
    ui->label_rotation->setText(QString::number(angle));
}

// 拼接采图
void MainWindow::capture_rotation() {
    // 采图
    cv::Mat opencvImage;
    // 开始运动
    if (!move_flag) {
        if (move_controll_connect())
            move_flag = true;
        else
            return;

        float value = 1;
        ZAux_Direct_SetTable(handle, 992, 1, &value);
    }

    // 等待接收和恢复图像，超时时间设置为5000ms.
    camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
    if (ptrGrabResult->GrabSucceeded()) {
        read_angles.push_back(angle);
        std::cout << QDateTime::currentDateTime().toString("hh:mm:ss.zz").toStdString()
                  << std::endl;
        std::cout << angle << std::endl;

        formatConverter.Convert(pylonImage, ptrGrabResult);  // 将抓取的缓冲数据转化成pylonImage
        opencvImage = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC1,
                              (uint8_t *)pylonImage.GetBuffer());  // 将pylonimage转成OpenCV image.

        cv::imwrite("./data/image/" + std::to_string(grabbed_images) + ".bmp", opencvImage);
        // qDebug() <<grabbed_images <<"  "<< ((double)cv::getTickCount() - time_start) /
        //        cv::getTickFrequency() * 1000 << "ms    "<<" angle "<<angle;
    }

    ui->image_timer_window->clear();
    QImage qimage = QImage((const uchar *)opencvImage.data, opencvImage.cols, opencvImage.rows,
                           QImage::Format_Grayscale8)
                        .rgbSwapped();
    ui->image_timer_window->setPixmap(QPixmap::fromImage(qimage));
    ui->image_timer_window->show();

    ++grabbed_images;

    // 判断旋转一周是否完成
    if (fabs(angle - 360.0) < 0.01) {
        messenges = "scan finished!";
        ui->message->append(messenges);

        photo_capture_timer->stop();
        read_angle_timer->stop();

        ui->open_camera->setEnabled(true);
        ui->close_camera->setEnabled(true);
        ui->splicing_rotation->setEnabled(true);
    }
}

bool MainWindow::move_controll_connect() {
    // 判断运动控制器是否连接成功
    bool flag;
    if (ZAux_OpenEth(ip_addr, &handle) != ERR_SUCCESS) {
        flag = false;
        QMessageBox *box = new QMessageBox(QMessageBox::Warning, "warning",
                                           "Motion controller connection failed!", QMessageBox::Ok);
        if (NULL != box->button(QMessageBox::Ok)) {
            box->button(QMessageBox::Ok)->setText("yes");
        }
        QTimer::singleShot(3000, box, SLOT(accept()));
        box->exec();
    } else {
        flag = true;
    }
    return flag;
}

/**
 * @brief image_processing 图像处理
 * @param i 图像序号
 */
void image_processing_rotation(int i, std::vector<int> &point_nums) {
    cv::Mat src_image, correct_image, dst_image;  // 原始图像，校正畸变后的图像和提取中心线后的图像
    std::vector<cv::Point2f> Pt;  // 激光条纹中心点校正后的像素坐标
    try {
        ImageProcessing::GetImage(i, src_image);                                 // 取得原图
        ImageProcessing::Correction(src_image, correct_image);                   // 图像校正
        ImageProcessing::CenterLineHorizontal(i, correct_image, dst_image, Pt);  // 提取中心线
        // ImageProcessing::CenterLineVertical(correct_image, dst_image, Pt);  // 提取中心线
        cv::imwrite("./data/center/" + std::to_string(i) + ".bmp", dst_image);
        ImageProcessing::Point2dto3dRotaion(i, Pt, read_angles);  // 计算三维轮廓坐标
        point_nums[i] = Pt.size();
    } catch (...) {
        std::cout << "image processing error!" << std::endl;
    }
}

/**
 * @brief image_processing_mutithread 多线程图像处理
 * @param n 图像序号
 * @param image_nums 图像数量
 */
void image_processing_rotation_mutiThread(int *n, int *image_nums, std::vector<int> *point_nums) {
    for (int i = *n; i < *n + *image_nums; ++i) {
        image_processing_rotation(i, *point_nums);
    }
}

void MainWindow::on_get_calibration_clicked() {
    int i = ui->image_i->text().toInt();
    int ret;
    //    if (i == 3 || i == 4) {
    //    ret = ImageProcessing::ConfigFileRead("./information/Hik_settings_1408_512.conf");
    //    } else {
    ret = ImageProcessing::ConfigFileRead("./information/pot_settings4.15.4.15.conf");
    //    }
    if (ret == -1) {
        QMessageBox *box = new QMessageBox(QMessageBox::Warning, "warning",
                                           "Calibration data cannot be obtained!", QMessageBox::Ok);
        if (NULL != box->button(QMessageBox::Ok)) {
            box->button(QMessageBox::Ok)->setText("yes");
        }
        QTimer::singleShot(3000, box, SLOT(accept()));
        box->exec();
        return;
    } else if (ret == 0) {
        QMessageBox *box = new QMessageBox(QMessageBox::Warning, "warning",
                                           "Calibration data error!", QMessageBox::Ok);
        if (NULL != box->button(QMessageBox::Ok)) {
            box->button(QMessageBox::Ok)->setText("yes");
        }
        QTimer::singleShot(3000, box, SLOT(accept()));
        box->exec();
        return;
    }
}

void MainWindow::on_capture_rotation_clicked() {
    ui->splicing_rotation->setEnabled(false);
    grabbed_images = 0;
    QString u, v, w, x, y, z;
    u = ui->gu->text();
    v = ui->gv->text();
    w = ui->gw->text();
    x = ui->gx->text();
    y = ui->gy->text();
    z = ui->gz->text();
    ImageProcessing::g_u = u.toDouble();
    ImageProcessing::g_v = v.toDouble();
    ImageProcessing::g_w = w.toDouble();
    ImageProcessing::g_x = x.toDouble();
    ImageProcessing::g_y = y.toDouble();
    ImageProcessing::g_z = z.toDouble();

    // 判断是否回零
    read_angle();
    if (fabs(angle - 360.0) < 0.01) {
        QMessageBox *box = new QMessageBox(QMessageBox::Warning, "warning",
                                           "Turntable does not return to zero!", QMessageBox::Ok);
        if (NULL != box->button(QMessageBox::Ok)) {
            box->button(QMessageBox::Ok)->setText("yes");
        }
        QTimer::singleShot(3000, box, SLOT(accept()));
        box->exec();
        return;
    }
    Pylon::PylonInitialize();  // 初始化Pylon对象

    // 判断相机连接情况
    if (!Pylon::CTlFactory::GetInstance().EnumerateDevices(device)) {
        QMessageBox *box = new QMessageBox(QMessageBox::Warning, "warning!",
                                           "Check camera connection!", QMessageBox::Ok);
        if (NULL != box->button(QMessageBox::Ok)) {
            box->button(QMessageBox::Ok)->setText("yes");
        }
        QTimer::singleShot(3000, box, SLOT(accept()));
        box->exec();
        return;
    }

    // 关联检测到的第一台相机设备
    camera.Attach(Pylon::CTlFactory::GetInstance().CreateDevice(device[0]));

    // 判断相机是否被其他程序占用
    try {
        camera.Open();  // 打开相机
    } catch (...) {
        QMessageBox *box = new QMessageBox(QMessageBox::Warning, "warning!",
                                           "The camera is already in use!", QMessageBox::Ok);
        if (NULL != box->button(QMessageBox::Ok)) {
            box->button(QMessageBox::Ok)->setText("yes");
        }

        QTimer::singleShot(3000, box, SLOT(accept()));
        box->exec();
        return;
    }

    camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);  // 相机默认设置连续抓取模式
    formatConverter.OutputPixelFormat = Pylon::PixelType_Mono8;  // 确定输出像素格式

    // 设置相机曝光时间
    GenApi::INodeMap &cameraNodeMap = camera.GetNodeMap();
    const GenApi::CFloatPtr exposureTime = cameraNodeMap.GetNode("ExposureTimeAbs");
    QString exposure;
    exposure = ui->exposure->text();
    // std::cout<<"angleTime.toInt()"<<exposure.toInt()<<std::endl;
    exposureTime->SetValue(exposure.toInt());

    // 采图定时器初始化
    photo_capture_timer = new QTimer(this);
    photo_capture_timer->setTimerType(Qt::PreciseTimer);
    connect(photo_capture_timer, SIGNAL(timeout()), this,
            SLOT(capture_rotation()));  // 定时采图
    QString capture;
    capture = ui->capture_timer->text();
    photo_capture_timer->start(capture.toInt());

    // 转角定时器初始化
    read_angle_timer = new QTimer(this);
    read_angle_timer->setTimerType(Qt::PreciseTimer);
    connect(read_angle_timer, SIGNAL(timeout()), this, SLOT(read_angle()));  // 定时
    QString angleTime;
    angleTime = ui->angle_timer->text();
    // std::cout<<"angleTime.toInt()"<<angleTime.toInt()<<std::endl;
    read_angle_timer->start(angleTime.toInt());
}

void MainWindow::on_splicing_rotation_clicked() {
    const int thread_nums = 12;                     // 图像处理线程数
    int image_nums = grabbed_images / thread_nums;  // 每个线程需要处理的图像数

    if (grabbed_images == 0) {
        QMessageBox *box = new QMessageBox(QMessageBox::Warning, "warning",
                                           "Please start scanning first!", QMessageBox::Ok);
        if (NULL != box->button(QMessageBox::Ok)) {
            box->button(QMessageBox::Ok)->setText("yes");
        }
        QTimer::singleShot(3000, box, SLOT(accept()));
        box->exec();
        return;
    }

    int n[thread_nums];
    std::thread t[thread_nums];
    std::vector<int> point_nums(grabbed_images);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_single(
        new pcl::PointCloud<pcl::PointXYZ>);  // 单帧点云

    float t_start = static_cast<float>(cv::getTickCount());
    // 多线程图像处理
    for (int i = 0; i < thread_nums; ++i) n[i] = image_nums * i;
    for (int i = 0; i < thread_nums; ++i)
        t[i] = std::thread(image_processing_rotation_mutiThread, &n[i], &image_nums, &point_nums);
    for (int i = 0; i < thread_nums; ++i) t[i].join();
    // 单线程处理其余图像
    for (int i = image_nums * thread_nums; i < grabbed_images; ++i)
        image_processing_rotation(i, point_nums);

    std::cout << "time of image processing "
              << ((float)cv::getTickCount() - t_start) / cv::getTickFrequency() * 1000 << "ms"
              << std::endl;
    messenges = "image processing finished!";
    ui->message->append(messenges);

    for (int i = 0; i < grabbed_images; ++i) {
        pcl::io::loadPCDFile("./data/cloud/" + std::to_string(i) + ".pcd", *cloud_single);
        size_t size = cloud_single->points.size();
        cloud->resize(cloud->points.size() + size);
        for (size_t j = 0; j < size; ++j) {
            cloud->points[cloud->points.size() - size + j] = cloud_single->points[j];
        }
    }

    std::cout << "time of get cloud "
              << ((float)cv::getTickCount() - t_start) / cv::getTickFrequency() * 1000 << "ms"
              << std::endl;
    std::cout << "points size:" << cloud->points.size() << std::endl;
    messenges = "points size:" + QString::number(cloud->points.size());
    ui->message->append(messenges);

    if (cloud->points.size() == 0) {
        QMessageBox *box = new QMessageBox(QMessageBox::Warning, "warning",
                                           "Point cloud not generated!", QMessageBox::Ok);
        if (NULL != box->button(QMessageBox::Ok)) {
            box->button(QMessageBox::Ok)->setText("yes");
        }
        QTimer::singleShot(3000, box, SLOT(accept()));
        box->exec();
        return;
    }

    pcl::io::savePCDFileBinary("cloud.pcd", *cloud);  // 存储.pcd格式点云文件

    // 显示点云
    //    viewer->removeAllPointClouds();
    //    viewer->addPointCloud(cloud, "cloud");
    //    ui->qvtkWidget2->SetRenderWindow(viewer->getRenderWindow());
    //    viewer->setupInteractor(ui->qvtkWidget->GetInteractor(),
    //    ui->qvtkWidget->GetRenderWindow()); ui->qvtkWidget2->update();

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();

    for (int i = 0; i < cloud->size(); i++) {
        vtkIdType pid[1];
        pid[0] = points->InsertNextPoint(cloud->at(i).x, cloud->at(i).y, cloud->at(i).z);
        vertices->InsertNextCell(1, pid);
    }

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetVerts(vertices);

    // 映射
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    // 演员
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(1.0, 1.0, 1.0);
    actor->GetProperty()->SetPointSize(1);

    vtkRenderer *renderer = vtkRenderer::New();
    renderer->AddActor(actor);
    renderer->ResetCamera();
    renderer->SetBackground(0, 0, 0);

    ui->qvtkWidget2->GetRenderWindow()->AddRenderer(renderer);
    ui->qvtkWidget2->GetRenderWindow()->Render();
}

// 线性拼接
/**
 * @brief read_turnTable_angle 读转角
 */
void MainWindow::read_move() {
    QString axis;
    axis = ui->axis->text();
    //    int ret = ZAux_Direct_GetDpos(handle,axis.toInt(), &distance);
    //    std::cout << QDateTime::currentDateTime().toString("hh:mm:ss.zz").toStdString() <<
    //    std::endl;
    // std::cout<<"distance "<<distance<<std::endl;
    ui->label_linear->setText(QString::number(distance));
}

void MainWindow::capture_linear() {
    cv::Mat opencvImage;      // 采图
    int axis[3] = {0, 1, 2};  // 运动轴
    QString ax = ui->axis->text();
    float mov[3] = {0, 0, 0};
    QString dis = ui->move_distance->text();
    // 线性运动
    QString sx, sy, sz, dx, dy, dz;
    sx = ui->move_start_x->text();
    sy = ui->move_start_y->text();
    sz = ui->move_start_z->text();
    dx = ui->move_dir_x->text();
    dy = ui->move_dir_y->text();
    dz = ui->move_dir_z->text();
    dirx = dx.toFloat();
    diry = dy.toFloat();
    dirz = dz.toFloat();
    // 开始运动
    if (!move_flag) {
        if (move_controll_connect())
            move_flag = true;
        else
            return;

        switch (ax.toInt()) {
            case 0:
                mov[0] = dx.toFloat() * dis.toFloat();
                break;
            case 1:
                mov[1] = dy.toFloat() * dis.toFloat();
                break;
            case 2:
                mov[2] = dz.toFloat() * dis.toFloat();
                break;
        }
        //        int ret = ZAux_Direct_Move(handle, 3, axis, mov);
    }
    MoveControl::safe(handle, axis, dis.toFloat());

    // 等待接收和恢复图像，超时时间设置为5000ms.
    camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
    if (ptrGrabResult->GrabSucceeded()) {
        read_moves.push_back(distance);
        std::cout << QDateTime::currentDateTime().toString("hh:mm:ss.zz").toStdString()
                  << std::endl;
        std::cout << distance << " " << read_moves[grabbed_images] << std::endl;

        formatConverter.Convert(pylonImage, ptrGrabResult);  // 将抓取的缓冲数据转化成pylonImage
        opencvImage = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC1,
                              (uint8_t *)pylonImage.GetBuffer());  // 将pylonimage转成OpenCV image.

        cv::imwrite("./data/image/" + std::to_string(grabbed_images) + ".bmp", opencvImage);
        // qDebug() <<grabbed_images <<"  "<< ((double)cv::getTickCount() - time_start) /
        //                                             cv::getTickFrequency() * 1000 << "ms "<<"
        //                                             angle "<<angle;
    }

    ui->image_timer_window->clear();
    QImage qimage = QImage((const uchar *)opencvImage.data, opencvImage.cols, opencvImage.rows,
                           QImage::Format_Grayscale8)
                        .rgbSwapped();
    ui->image_timer_window->setPixmap(QPixmap::fromImage(qimage));
    ui->image_timer_window->show();

    ++grabbed_images;

    // 判断是否运动到终点
    double endPosition;
    switch (ax.toInt()) {
        case 0:
            endPosition = sx.toDouble() + dx.toDouble() * dis.toDouble();
            break;
        case 1:
            endPosition = sy.toDouble() + dy.toDouble() * dis.toDouble();
            break;
        case 2:
            endPosition = sz.toDouble() + dz.toDouble() * dis.toDouble();
            break;
    }

    std::cout << "endPosition " << endPosition << std::endl;

    if (fabs(distance - endPosition) < 0.01) {
        messenges = "scan finished!";
        ui->message->append(messenges);

        int ret = ZAux_Close(handle);
        ret = ZAux_Direct_Single_Cancel(handle, ax.toInt(), 3);
        handle = NULL;

        photo_capture_timer->stop();
        read_move_timer->stop();

        ui->open_camera->setEnabled(true);
        ui->close_camera->setEnabled(true);
        ui->splicing_linear->setEnabled(true);
    }
}

/**
 * @brief image_processing_linear 图像处理
 * @param i 图像序号
 * @param point_nums
 */
void image_processing_linear(int i, std::vector<int> &point_nums) {
    cv::Mat src_image, correct_image,
        dst_image;  // 原始图像，校正畸变后的图像和提取中心线后的图像
    std::vector<cv::Point2f> Pt;  // 激光条纹中心点校正后的像素坐标
    std::vector<float> dir;
    dir.resize(3);
    dir[0] = dirx;
    dir[1] = diry;
    dir[2] = dirz;

    try {
        ImageProcessing::GetImage(i, src_image);                                 // 取得原图
        ImageProcessing::Correction(src_image, correct_image);                   // 图像校正
        ImageProcessing::CenterLineHorizontal(i, correct_image, dst_image, Pt);  // 提取中心线
        cv::imwrite("./data/center/" + std::to_string(i) + ".bmp", dst_image);
        // CenterLineVertical(correct_image, dst_image, Pt);  // 提取中心线
        ImageProcessing::Point2dto3dLinear(i, Pt, read_moves, dir);  // 计算三维轮廓坐标
        point_nums[i] = Pt.size();
    } catch (...) {
        std::cout << "image processing error!" << std::endl;
    }
}

/**
 * @brief image_processing_mutithread 多线程图像处理
 * @param n 图像序号
 * @param image_nums 图像数量
 */
void image_processing_linear_mutiThread(int *n, int *image_nums, std::vector<int> *point_nums) {
    for (int i = *n; i < *n + *image_nums; ++i) {
        image_processing_linear(i, *point_nums);
    }
}

void MainWindow::on_capture_linear_clicked() {
    QString axis;
    axis = ui->axis->text();
    // std::cout<<"axis"<<axis.toInt()<<std::endl;
    ui->splicing_linear->setEnabled(false);
    grabbed_images = 0;

    // 运动起点
    QString sx, sy, sz;
    sx = ui->move_start_x->text();
    sy = ui->move_start_y->text();
    sz = ui->move_start_z->text();
    switch (axis.toInt()) {
        case 0:
            startPosition = sx.toDouble();
            break;
        case 1:
            startPosition = sy.toDouble();
            break;
        case 2:
            startPosition = sz.toDouble();
            break;
    }
    // std::cout<<"startPosition "<<startPosition<<std::endl;

    // 判断是否到达线性扫描起始位置
    //    int ret = move_controll_connect();
    read_move();
    //    int axis2[3]={0,1,2};
    //    float mov[3]={sx.toDouble(),sy.toDouble(),sz.toDouble()};
    //    move(handle, 3, axis2, mov);

    if (fabs(distance - startPosition) > 0.01) {
        QMessageBox *box = new QMessageBox(QMessageBox::Warning, "warning",
                                           "Scan start position not reached!", QMessageBox::Ok);
        if (NULL != box->button(QMessageBox::Ok)) {
            box->button(QMessageBox::Ok)->setText("yes");
        }
        QTimer::singleShot(3000, box, SLOT(accept()));
        box->exec();
        return;
    }
    Pylon::PylonInitialize();  // 初始化Pylon对象

    // 判断相机连接情况
    if (!Pylon::CTlFactory::GetInstance().EnumerateDevices(device)) {
        QMessageBox *box = new QMessageBox(QMessageBox::Warning, "warning!",
                                           "Check camera connection!", QMessageBox::Ok);
        if (NULL != box->button(QMessageBox::Ok)) {
            box->button(QMessageBox::Ok)->setText("yes");
        }
        QTimer::singleShot(3000, box, SLOT(accept()));
        box->exec();
        return;
    }

    camera.Attach(
        Pylon::CTlFactory::GetInstance().CreateDevice(device[0]));  // 关联检测到的第一台相机设备

    // 判断相机是否被其他程序占用
    try {
        camera.Open();  // 打开相机
    } catch (...) {
        QMessageBox *box = new QMessageBox(QMessageBox::Warning, "warning!",
                                           "The camera is already in use!", QMessageBox::Ok);
        if (NULL != box->button(QMessageBox::Ok)) {
            box->button(QMessageBox::Ok)->setText("yes");
        }

        QTimer::singleShot(3000, box, SLOT(accept()));
        box->exec();
        return;
    }

    camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);  // 相机默认设置连续抓取模式
    formatConverter.OutputPixelFormat = Pylon::PixelType_Mono8;  // 确定输出像素格式

    // 设置相机曝光时间
    GenApi::INodeMap &cameraNodeMap = camera.GetNodeMap();
    const GenApi::CFloatPtr exposureTime = cameraNodeMap.GetNode("ExposureTimeAbs");
    QString exposure;
    exposure = ui->exposure->text();
    // std::cout<<"angleTime.toInt()"<<exposure.toInt()<<std::endl;
    exposureTime->SetValue(exposure.toInt());

    // 采图定时器初始化
    photo_capture_timer = new QTimer(this);
    photo_capture_timer->setTimerType(Qt::PreciseTimer);
    connect(photo_capture_timer, SIGNAL(timeout()), this, SLOT(capture_linear()));  // 定时采图
    QString capture;
    capture = ui->capture_timer->text();
    photo_capture_timer->start(capture.toInt());

    // 位置定时器初始化
    read_move_timer = new QTimer(this);
    read_move_timer->setTimerType(Qt::PreciseTimer);
    connect(read_move_timer, SIGNAL(timeout()), this, SLOT(read_move()));  // 定时
    QString moveTime;
    moveTime = ui->move_timer->text();
    // std::cout<<"moveTime.toInt()"<<moveTime.toInt()<<std::endl;
    read_move_timer->start(moveTime.toInt());
}

void MainWindow::on_splicing_linear_clicked() {
    const int thread_nums = 12;                     // 图像处理线程数
    int image_nums = grabbed_images / thread_nums;  // 每个线程需要处理的图像数

    if (grabbed_images == 0) {
        QMessageBox *box = new QMessageBox(QMessageBox::Warning, "warning",
                                           "Please start scanning first!", QMessageBox::Ok);
        if (NULL != box->button(QMessageBox::Ok)) {
            box->button(QMessageBox::Ok)->setText("yes");
        }
        QTimer::singleShot(3000, box, SLOT(accept()));
        box->exec();
        return;
    }

    int n[thread_nums];
    std::thread t[thread_nums];
    std::vector<int> point_nums(grabbed_images);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_single(
        new pcl::PointCloud<pcl::PointXYZ>);  // 单帧点云

    float t_start = static_cast<float>(cv::getTickCount());
    // 多线程图像处理
    for (int i = 0; i < thread_nums; ++i) n[i] = image_nums * i;
    for (int i = 0; i < thread_nums; ++i)
        t[i] = std::thread(image_processing_linear_mutiThread, &n[i], &image_nums, &point_nums);
    for (int i = 0; i < thread_nums; ++i) t[i].join();
    // 单线程处理其余图像
    for (int i = image_nums * thread_nums; i < grabbed_images; ++i)
        image_processing_linear(i, point_nums);

    std::cout << "time of image processing "
              << ((float)cv::getTickCount() - t_start) / cv::getTickFrequency() * 1000 << "ms"
              << std::endl;
    messenges = "image processing finished!";
    ui->message->append(messenges);

    for (int i = 0; i < grabbed_images; ++i) {
        pcl::io::loadPCDFile("./data/cloud/" + std::to_string(i) + ".pcd", *cloud_single);
        size_t size = cloud_single->points.size();
        cloud->resize(cloud->points.size() + size);
        for (size_t j = 0; j < size; ++j) {
            cloud->points[cloud->points.size() - size + j] = cloud_single->points[j];
        }
    }

    std::cout << "time of get cloud "
              << ((float)cv::getTickCount() - t_start) / cv::getTickFrequency() * 1000 << "ms"
              << std::endl;
    std::cout << "points size:" << cloud->points.size() << std::endl;
    messenges = "points size:" + QString::number(cloud->points.size());
    ui->message->append(messenges);

    if (cloud->points.size() == 0) {
        QMessageBox *box = new QMessageBox(QMessageBox::Warning, "warning",
                                           "Point cloud not generated!", QMessageBox::Ok);
        if (NULL != box->button(QMessageBox::Ok)) {
            box->button(QMessageBox::Ok)->setText("yes");
        }
        QTimer::singleShot(3000, box, SLOT(accept()));
        box->exec();
        return;
    }

    pcl::io::savePCDFileBinary("cloud.pcd", *cloud);  // 存储.pcd格式点云文件

    // 显示点云
    //    viewer->removeAllPointClouds();
    //    viewer->addPointCloud(cloud, "cloud");
    //    ui->qvtkWidget2->SetRenderWindow(viewer->getRenderWindow());
    //    viewer->setupInteractor(ui->qvtkWidget->GetInteractor(),
    //    ui->qvtkWidget->GetRenderWindow()); ui->qvtkWidget2->update();

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();

    for (int i = 0; i < cloud->size(); i++) {
        vtkIdType pid[1];
        pid[0] = points->InsertNextPoint(cloud->at(i).x, cloud->at(i).y, cloud->at(i).z);
        vertices->InsertNextCell(1, pid);
    }

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetVerts(vertices);

    // 映射
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    // 演员
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(1.0, 1.0, 1.0);
    actor->GetProperty()->SetPointSize(1);

    vtkRenderer *renderer = vtkRenderer::New();
    renderer->AddActor(actor);
    renderer->ResetCamera();
    renderer->SetBackground(0, 0, 0);

    ui->qvtkWidget2->GetRenderWindow()->AddRenderer(renderer);
    ui->qvtkWidget2->GetRenderWindow()->Render();
}

void MainWindow::on_btn_calibration_clicked() {
    config *configWindow = new config;
    configWindow->show();
}
