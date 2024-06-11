// #ifndef STABLE_H
// #define STABLE_H

#include <QVTKWidget.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <pylon/PylonIncludes.h>
#include <pylon/gige/BaslerGigECamera.h>
#include <vtkAutoInit.h>
#include <vtkRenderWindow.h>

#include <QDebug>
#include <QDir>
#include <QElapsedTimer>
#include <QMainWindow>
#include <QTimer>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "config/config.h"
#include "image_processing/image_processing.h"
#include "image_processing/normalcentroid.h"
#include "image_processing/steger.h"
#include "lib/zauxdll2.h"
#include "lib/zmotion.h"
#include "qdatetime.h"

// #endif // STABLE_H
