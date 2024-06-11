QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

QMAKE_CXXFLAGS += /MP

# Use Precompiled headers (PCH)
CONFIG += precompile_header
PRECOMPILED_HEADER = stable.h

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

QMAKE_CXXFLAGS_RELEASE = -O3 -ZI -MD
QMAKE_LFLAGS_RELEASE = /DEBUG

SOURCES += \
    config/camera_calibration.cpp \
    config/config.cpp \
    config/myLable.cpp \
    image_processing/image_processing.cpp \
    image_processing/normalcentroid.cpp \
    image_processing/steger.cpp \
    main.cpp \
    mainwindow.cpp \
    move_control/move_control.cpp

HEADERS += \
    config/camera_calibration.h \
    config/config.h \
    config/myLable.h \
    image_processing/image_processing.h \
    image_processing/normalcentroid.h \
    image_processing/steger.h \
    mainwindow.h \
    move_control/move_control.h \
    stable.h \
    lib/zauxdll2.h \
    lib/zmotion.h

FORMS += \
    config/config.ui \
    mainwindow.ui


INCLUDEPATH += D:/opencv4.6.0/opencv/build/include
CONFIG(debug, debug|release){
    LIBS += D:/opencv4.6.0/opencv/build/x64/vc15/lib/opencv_world460d.lib
} else {
    LIBS += D:/opencv4.6.0/opencv/build/x64/vc15/lib/opencv_world460.lib
}

CONFIG(debug, debug|release){
    INCLUDEPATH += $$quote(D:/ProgramData/vtk_8.2.0_debug/include/vtk-8.2)
    LIBS +=  $$quote(D:/ProgramData/vtk_8.2.0_debug/lib/vtk*.lib)
} else {
    INCLUDEPATH += $$quote(D:/ProgramData/vtk_8.2.0_release/include/vtk-8.2)
    LIBS +=  $$quote(D:/ProgramData/vtk_8.2.0_release/lib/vtk*.lib)
}

INCLUDEPATH += D:\PCL1.9.1\3rdParty\OpenNI2\Include\
INCLUDEPATH += D:\PCL1.9.1\include\pcl-1.9\pcl
INCLUDEPATH += D:\PCL1.9.1\include\pcl-1.9\
INCLUDEPATH += D:\PCL1.9.1\3rdParty\Boost\include\boost-1_68\
INCLUDEPATH += D:\PCL1.9.1\3rdParty\Eigen\eigen3\
INCLUDEPATH += D:\PCL1.9.1\3rdParty\FLANN\include\
INCLUDEPATH += D:\PCL1.9.1\3rdParty\FLANN\include\flann\
INCLUDEPATH += D:\PCL1.9.1\3rdParty\Qhull\include\
INCLUDEPATH += D:\PCL1.9.1\3rdParty\VTK\include\vtk-8.1
include(D:\PCL1.9.1\pcl191.pri)


INCLUDEPATH += $$quote(D:/Basler/pylon7/Development/include)
INCLUDEPATH += $$quote(D:/Basler/pylon7/Development/Samples/C++/include)
LIBS +=  $$quote(D:/Basler/pylon7/Development/lib/x64/*.lib)


# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

#win32: LIBS += -L$$PWD/./ -lzauxdll

INCLUDEPATH += $$PWD/.
DEPENDPATH += $$PWD/.

#win32:!win32-g++: PRE_TARGETDEPS += $$PWD/./lib/zauxdll.lib
#else:win32-g++: PRE_TARGETDEPS += $$PWD/./libzauxdll.a

#win32: LIBS += -L$$PWD/./ -lzmotion

#win32:!win32-g++: PRE_TARGETDEPS += $$PWD/./lib/zmotion.lib
#else:win32-g++: PRE_TARGETDEPS += $$PWD/./libzmotion.a

DISTFILES += \
    lib/zauxdll.lib \
    lib/zmotion.lib

OBJECTS_DIR = build/obj
MOC_DIR = build/moc
UI_DIR = build/ui
RCC_DIR = build/qrc
#DESTDIR = build/bin


