QT -= core
QT -= gui

#TARGET = YSU_2021_Infantry
TARGET =YSU_2022_Sentry
CONFIG += console
CONFIG -= app_bundle
CONFIG += c++14
TEMPLATE = app


INCLUDEPATH += /usr/local/include \
/usr/local/include/opencv \
/usr/local/include/opencv2 \
#/home/nano/linuxSDK_V2.1.0.2/include \
/home/robomaster/linuxSDK_V2.1.0.2/include \
/usr/local/include/gsl \
# 需要自行修改的库
/home/robomaster/intel/openvino/runtime/include \
/home/robomaster/intel/openvino/runtime/include/ie \
/home/robomaster/intel/openvino/runtime/include/ngraph \
/home/robomaster/intel/openvino/runtime/include/openvino

LIBS += /usr/local/lib/libopencv_calib3d.so \
/usr/local/lib/libopencv_core.so \
/usr/local/lib/libopencv_features2d.so \
/usr/local/lib/libopencv_flann.so \
/usr/local/lib/libopencv_highgui.so \
/usr/local/lib/libopencv_imgcodecs.so \
/usr/local/lib/libopencv_imgproc.so \
/usr/local/lib/libopencv_ml.so \
/usr/local/lib/libopencv_objdetect.so \
/usr/local/lib/libopencv_photo.so \
/usr/local/lib/libopencv_shape.so \
/usr/local/lib/libopencv_stitching.so \
/usr/local/lib/libopencv_superres.so \
/usr/local/lib/libopencv_videoio.so \
/usr/local/lib/libopencv_video.so \
/usr/local/lib/libopencv_videostab.so\
/home/robomaster/linuxSDK_V2.1.0.2/lib/x64/libMVSDK.so \
/usr/local/lib/libgsl.so \
/usr/local/lib/libgslcblas.so \
# 自行根据微机的库的位置修改
/home/robomaster/intel/openvino/runtime/lib/intel64/libopenvino.so \
-L/home/robomaster/intel/openvino/runtime/lib/intel64 -lopenvino
#                                                -lngraph \
#                                                -ltbb \


HEADERS += \
    ArmorTrack/armor_track.h \
    ArmorTrack/auto_aim.h \
    Communication/SerialPort.hpp \
    DeepLearning/yolov5.h \
    GafSolver/gaf_projectile_solver.h \
    GafSolver/gimbal_transform_tool.h \
    GafSolver/iterative_projectile_tool.h \
    GafSolver/projectile_solver_interface.h \
    Kalman/Predictor.h \
    Kalman/SpinDetector.h \
    Kalman/kalman3d.h \
    ThreadManager/thread_manager.h \
    ArmorDetector/armor_detector.h \
    RuneDetector/rune_detector.h \
    CameraManager/camera_manager.h \
    Classify/classify.h \
    Communication/communication.h \
    Pose/angle_solver.h \
    Main/headfiles.h \
    Timer/Timer.h \
    Timer/Log.h \
    Forecast/forecast.h \
    Kalman/kalman.h \
    SVM/svm.h \
    Forecast/ysu_gsl.h
 #   Forecast/PID.h \




SOURCES += \
    ArmorTrack/armor_track.cpp \
    ArmorTrack/auto_aim.cpp \
    Communication/SerialPort.cpp \
    DeepLearning/yolov5.cpp \
    GafSolver/gaf_projectile_solver.cpp \
    GafSolver/gimbal_transform_tool.cpp \
    GafSolver/iterative_projectile_tool.cpp \
    Kalman/Predictor.cpp \
    Kalman/SpinDetector.cpp \
    Kalman/kalman3d.cpp \
    ThreadManager/thread_manager.cpp \
    ArmorDetector/armor_detector.cpp \
    RuneDetector/rune_detector.cpp \
    CameraManager/camera_manager.cpp \
    Classify/classify.cpp \
    Communication/communication.cpp \
    Main/main.cpp \
    Pose/angle_solver.cpp \
    Timer/Timer.cpp \
    Forecast/forecast.cpp \
    Kalman/kalman.cpp \
    SVM/svm.cpp \
    Forecast/ysu_gsl.cpp
 #   Forecast/PID.cpp \





