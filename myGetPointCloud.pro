QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

QMAKE_CXXFLAGS += /MP

# Use Precompiled headers (PCH)
#CONFIG += precompile_header
#PRECOMPILED_HEADER = stable.h
#HEADERS += stable.h \
#    lib/zauxdll2.h \
#    lib/zmotion.h

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


#CONFIG(debug, debug|release){
#    INCLUDEPATH += $$quote(D:/ProgramData/vtk_8.2.0_debug/include/vtk-8.2)
#    LIBS +=  $$quote(D:/ProgramData/vtk_8.2.0_debug/lib/vtk*.lib)
#} else {
#    INCLUDEPATH += $$quote(D:/ProgramData/vtk_8.2.0_release/include/vtk-8.2)
#    LIBS +=  $$quote(D:/ProgramData/vtk_8.2.0_release/lib/vtk*.lib)
#}


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
#CONFIG(debug, debug|release){
#LIBS += -LD:\PCL1.9.1\lib\
#            -lpcl_common_debug
#            -lpcl_features_debug
#            -lpcl_filters_debug
#            -lpcl_io_debug
#            -lpcl_io_ply_debug
#            -lpcl_kdtree_debug
#            -lpcl_keypoints_debug
#            -lpcl_ml_debug
#            -lpcl_octree_debug
#            -lpcl_outofcore_debug
#            -lpcl_people_debug
#            -lpcl_recognition_debug
#            -lpcl_registration_debug
#            -lpcl_sample_consensus_debug
#            -lpcl_search_debug
#            -lpcl_segmentation_debug
#            -lpcl_stereo_debug
#            -lpcl_surface_debug
#            -lpcl_tracking_debug
#            -lpcl_visualization_debug
#LIBS += D:\PCL1.9.1\lib\pcl_common_debug.lib\
#        D:\PCL1.9.1\lib\pcl_features_debug.lib\
#        D:\PCL1.9.1\lib\pcl_filters_debug.lib\
#        D:\PCL1.9.1\lib\pcl_io_debug.lib\
#        D:\PCL1.9.1\lib\pcl_io_ply_debug.lib\
#        D:\PCL1.9.1\lib\pcl_kdtree_debug.lib\
#        D:\PCL1.9.1\lib\pcl_keypoints_debug.lib\
#        D:\PCL1.9.1\lib\pcl_ml_debug.lib\
#        D:\PCL1.9.1\lib\pcl_octree_debug.lib\
#        D:\PCL1.9.1\lib\pcl_outofcore_debug.lib\
#        D:\PCL1.9.1\lib\pcl_people_debug.lib\
#        D:\PCL1.9.1\lib\pcl_recognition_debug.lib\
#        D:\PCL1.9.1\lib\pcl_registration_debug.lib\
#        D:\PCL1.9.1\lib\pcl_sample_consensus_debug.lib\
#        D:\PCL1.9.1\lib\pcl_search_debug.lib\
#        D:\PCL1.9.1\lib\pcl_segmentation_debug.lib\
#        D:\PCL1.9.1\lib\pcl_stereo_debug.lib\
#        D:\PCL1.9.1\lib\pcl_surface_debug.lib\
#        D:\PCL1.9.1\lib\pcl_tracking_debug.lib\
#        D:\PCL1.9.1\lib\pcl_visualization_debug.lib
#LIBS+=  D:\PCL1.9.1\3rdParty\Boost\lib\libboost_atomic-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_bzip2-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_chrono-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_container-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_context-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_contract-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_coroutine-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_date_time-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_exception-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_fiber-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_filesystem-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_graph-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_graph_parallel-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_iostreams-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_locale-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_log-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_log_setup-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_math_c99-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_math_c99f-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_math_c99l-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_math_tr1-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_math_tr1f-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_math_tr1l-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_mpi-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_numpy27-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_numpy37-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_prg_exec_monitor-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_program_options-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_python27-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_python37-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_random-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_regex-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_serialization-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_signals-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_stacktrace_noop-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_stacktrace_windbg-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_stacktrace_windbg_cached-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_system-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_test_exec_monitor-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_thread-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_timer-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_type_erasure-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_unit_test_framework-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_wave-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_wserialization-vc141-mt-gd-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_zlib-vc141-mt-gd-x64-1_68.lib
#LIBS+=  D:\PCL1.9.1\3rdParty\FLANN\lib\flann_cpp_s-gd.lib\
#        D:\PCL1.9.1\3rdParty\FLANN\lib\flann_s-gd.lib\
#        D:\PCL1.9.1\3rdParty\FLANN\lib\flann-gd.lib
#LIBS+=  D:\PCL1.9.1\3rdParty\Qhull\lib\qhullstatic_d.lib\
#        D:\PCL1.9.1\3rdParty\Qhull\lib\qhull_d.lib\
#        D:\PCL1.9.1\3rdParty\Qhull\lib\qhull_p_d.lib\
#        D:\PCL1.9.1\3rdParty\Qhull\lib\qhullcpp_d.lib\
#        D:\PCL1.9.1\3rdParty\Qhull\lib\qhullstatic_r_d.lib
#LIBS+=  D:\PCL1.9.1\3rdParty\VTK\lib\vtkalglib-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkChartsCore-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkCommonColor-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkCommonComputationalGeometry-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkCommonCore-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkCommonDataModel-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkCommonExecutionModel-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkCommonMath-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkCommonMisc-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkCommonSystem-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkCommonTransforms-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkDICOMParser-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkDomainsChemistry-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkexoIIc-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkexpat-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersAMR-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersCore-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersExtraction-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersFlowPaths-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersGeneral-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersGeneric-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersGeometry-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersHybrid-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersHyperTree-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersImaging-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersModeling-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersParallel-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersParallelImaging-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersPoints-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersProgrammable-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersSelection-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersSMP-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersSources-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersStatistics-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersTexture-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersTopology-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersVerdict-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkfreetype-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkGeovisCore-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkgl2ps-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkhdf5-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkhdf5_hl-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkImagingColor-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkImagingCore-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkImagingFourier-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkImagingGeneral-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkImagingHybrid-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkImagingMath-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkImagingMorphological-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkImagingSources-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkImagingStatistics-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkImagingStencil-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkInfovisCore-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkInfovisLayout-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkInteractionImage-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkInteractionStyle-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkInteractionWidgets-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOAMR-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOCore-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOEnSight-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOExodus-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOExport-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOExportOpenGL-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOGeometry-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOImage-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOImport-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOInfovis-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOLegacy-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOLSDyna-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOMINC-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOMovie-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIONetCDF-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOParallel-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOParallelXML-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOPLY-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOSQL-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOTecplotTable-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOVideo-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOXML-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOXMLParser-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkjpeg-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkjsoncpp-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtklibharu-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtklibxml2-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtklz4-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkmetaio-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkNetCDF-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtknetcdfcpp-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkoggtheora-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkParallelCore-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkpng-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkproj4-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkRenderingAnnotation-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkRenderingContext2D-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkRenderingContextOpenGL-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkRenderingCore-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkRenderingFreeType-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkRenderingGL2PS-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkRenderingImage-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkRenderingLabel-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkRenderingLIC-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkRenderingLOD-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkRenderingOpenGL-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkRenderingVolume-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkRenderingVolumeOpenGL-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtksqlite-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtksys-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtktiff-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkverdict-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkViewsContext2D-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkViewsCore-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkViewsInfovis-8.1-gd.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkzlib-8.1-gd.lib\
#LIBS+=  D:\PCL1.9.1\3rdParty\OpenNI2\Lib\OpenNI2.lib
#}
#else {
#LIBS += -LD:\PCL1.9.1\lib\
#            -lpcl_common_release \
#            -lpcl_features_release \
#            -lpcl_filters_release \
#            -lpcl_io_release \
#            -lpcl_io_ply_release \
#            -lpcl_kdtree_release \
#            -lpcl_keypoints_release \
#            -lpcl_ml_release \
#            -lpcl_octree_release \
#            -lpcl_outofcore_release \
#            -lpcl_people_release \
#            -lpcl_recognition_release \
#            -lpcl_registration_release \
#            -lpcl_sample_consensus_release \
#            -lpcl_search_release \
#            -lpcl_segmentation_release \
#            -lpcl_stereo_release \
#            -lpcl_surface_release \
#            -lpcl_tracking_release \
#            -lpcl_visualization_release
#LIBS += D:\PCL1.9.1\lib\pcl_common_release.lib\
#        D:\PCL1.9.1\lib\pcl_features_release.lib\
#        D:\PCL1.9.1\lib\pcl_filters_release.lib\
#        D:\PCL1.9.1\lib\pcl_io_release.lib\
#        D:\PCL1.9.1\lib\pcl_io_ply_release.lib\
#        D:\PCL1.9.1\lib\pcl_kdtree_release.lib\
#        D:\PCL1.9.1\lib\pcl_keypoints_release.lib\
#        D:\PCL1.9.1\lib\pcl_ml_release.lib\
#        D:\PCL1.9.1\lib\pcl_octree_release.lib\
#        D:\PCL1.9.1\lib\pcl_outofcore_release.lib\
#        D:\PCL1.9.1\lib\pcl_people_release.lib\
#        D:\PCL1.9.1\lib\pcl_recognition_release.lib\
#        D:\PCL1.9.1\lib\pcl_registration_release.lib\
#        D:\PCL1.9.1\lib\pcl_sample_consensus_release.lib\
#        D:\PCL1.9.1\lib\pcl_search_release.lib\
#        D:\PCL1.9.1\lib\pcl_segmentation_release.lib\
#        D:\PCL1.9.1\lib\pcl_stereo_release.lib\
#        D:\PCL1.9.1\lib\pcl_surface_release.lib\
#        D:\PCL1.9.1\lib\pcl_tracking_release.lib\
#        D:\PCL1.9.1\lib\pcl_visualization_release.lib
#LIBS+=  D:\PCL1.9.1\3rdParty\Boost\lib\libboost_atomic-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_bzip2-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_chrono-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_container-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_context-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_contract-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_coroutine-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_date_time-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_exception-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_fiber-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_filesystem-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_graph-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_graph_parallel-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_iostreams-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_locale-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_log-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_log_setup-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_math_c99-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_math_c99f-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_math_c99l-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_math_tr1-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_math_tr1f-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_math_tr1l-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_mpi-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_numpy27-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_numpy37-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_prg_exec_monitor-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_program_options-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_python27-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_python37-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_random-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_regex-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_serialization-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_signals-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_stacktrace_noop-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_stacktrace_windbg-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_stacktrace_windbg_cached-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_system-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_test_exec_monitor-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_thread-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_timer-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_type_erasure-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_unit_test_framework-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_wave-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_wserialization-vc141-mt-x64-1_68.lib\
#        D:\PCL1.9.1\3rdParty\Boost\lib\libboost_zlib-vc141-mt-x64-1_68.lib
#LIBS+=  D:\PCL1.9.1\3rdParty\FLANN\lib\flann_cpp_s.lib\
#        D:\PCL1.9.1\3rdParty\FLANN\lib\flann_s.lib\
#        D:\PCL1.9.1\3rdParty\FLANN\lib\flann.lib
#LIBS+=  D:\PCL1.9.1\3rdParty\Qhull\lib\qhullstatic.lib\
#        D:\PCL1.9.1\3rdParty\Qhull\lib\qhull.lib\
#        D:\PCL1.9.1\3rdParty\Qhull\lib\qhull_p.lib\
#        D:\PCL1.9.1\3rdParty\Qhull\lib\qhullcpp.lib\
#        D:\PCL1.9.1\3rdParty\Qhull\lib\qhullstatic_r_d.lib
#LIBS+=  D:\PCL1.9.1\3rdParty\VTK\lib\vtkalglib-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkChartsCore-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkCommonColor-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkCommonComputationalGeometry-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkCommonCore-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkCommonDataModel-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkCommonExecutionModel-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkCommonMath-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkCommonMisc-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkCommonSystem-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkCommonTransforms-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkDICOMParser-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkDomainsChemistry-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkexoIIc-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkexpat-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersAMR-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersCore-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersExtraction-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersFlowPaths-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersGeneral-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersGeneric-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersGeometry-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersHybrid-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersHyperTree-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersImaging-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersModeling-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersParallel-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersParallelImaging-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersPoints-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersProgrammable-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersSelection-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersSMP-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersSources-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersStatistics-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersTexture-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersTopology-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkFiltersVerdict-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkfreetype-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkGeovisCore-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkgl2ps-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkhdf5-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkhdf5_hl-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkImagingColor-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkImagingCore-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkImagingFourier-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkImagingGeneral-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkImagingHybrid-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkImagingMath-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkImagingMorphological-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkImagingSources-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkImagingStatistics-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkImagingStencil-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkInfovisCore-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkInfovisLayout-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkInteractionImage-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkInteractionStyle-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkInteractionWidgets-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOAMR-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOCore-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOEnSight-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOExodus-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOExport-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOExportOpenGL-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOGeometry-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOImage-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOImport-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOInfovis-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOLegacy-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOLSDyna-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOMINC-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOMovie-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIONetCDF-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOParallel-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOParallelXML-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOPLY-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOSQL-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOTecplotTable-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOVideo-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOXML-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkIOXMLParser-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkjpeg-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkjsoncpp-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtklibharu-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtklibxml2-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtklz4-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkmetaio-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkNetCDF-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtknetcdfcpp-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkoggtheora-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkParallelCore-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkpng-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkproj4-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkRenderingAnnotation-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkRenderingContext2D-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkRenderingContextOpenGL-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkRenderingCore-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkRenderingFreeType-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkRenderingGL2PS-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkRenderingImage-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkRenderingLabel-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkRenderingLIC-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkRenderingLOD-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkRenderingOpenGL-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkRenderingVolume-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkRenderingVolumeOpenGL-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtksqlite-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtksys-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtktiff-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkverdict-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkViewsContext2D-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkViewsCore-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkViewsInfovis-8.1.lib\
#        D:\PCL1.9.1\3rdParty\VTK\lib\vtkzlib-8.1.lib
#LIBS+=  D:\PCL1.9.1\3rdParty\OpenNI2\Lib\OpenNI2.lib
#}


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

win32:!win32-g++: PRE_TARGETDEPS += $$PWD/./lib/zauxdll.lib
else:win32-g++: PRE_TARGETDEPS += $$PWD/./libzauxdll.a

#win32: LIBS += -L$$PWD/./ -lzmotion

#INCLUDEPATH += $$PWD/.
#DEPENDPATH += $$PWD/.

win32:!win32-g++: PRE_TARGETDEPS += $$PWD/./lib/zmotion.lib
else:win32-g++: PRE_TARGETDEPS += $$PWD/./libzmotion.a

DISTFILES += \
    lib/zauxdll.lib \
    lib/zmotion.lib
