#-------------------------------------------------
#
# Project created by QtCreator 2017-08-16T09:20:14
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Drosophila_Tracker
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        main.cpp \
        main_window.cpp \
    simple_tracker.cpp \
    contour_extractor.cpp \
    basic_calc.cpp \
    larva_model.cpp \
    rjmcmc_target.cpp \
    observation_prob.cpp \
    motion_model.cpp \
    motion_prior.cpp \
    rjmcmc_tracker.cpp \
    system_state.cpp

HEADERS += \
        main_window.h \
    simple_tracker.h \
    consts.h \
    contour_extractor.h \
    basic_calc.h \
    larva_model.h \
    rjmcmc_target.h \
    observation_prob.h \
    motion_model.h \
    motion_prior.h \
    rjmcmc_tracker.h \
    system_state.h

FORMS += \
        main_window.ui

INCLUDEPATH +=/home/tmichels/local_home/opt/opencv/include
INCLUDEPATH +=/home/tmichels/local_home/opt/opencv/include/opencv
INCLUDEPATH +=/home/tmichels/local_home/opt/opencv/include/opencv2

LIBS += -L/home/tmichels/local_home/opt/opencv/lib \
-lopencv_highgui \
-lopencv_imgcodecs \
-lopencv_core \
-lopencv_imgproc
