#-------------------------------------------------
#
# Project created by QtCreator 2014-02-11T10:41:29
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = OmniWheel
CONFIG   += console
CONFIG   -= app_bundle

CONFIG(debug, debug|release) {
    DESTDIR = ../PODO_PROC_Build
} else {
    DESTDIR = ../PODO_PROC_Build
}

TEMPLATE = app



INCLUDEPATH += \


LIBS        += \
    -lpthread \
    -lrt \
   -L../../SHARE/Libs       -lKINE_DRC_HUBO2 \
   -L../../SHARE/Libs	-lik_math2 \

SOURCES += main.cpp \
    joint.cpp \
    BasicTrajectory.cpp \
    taskmotion.cpp \
    ManualCAN.cpp

HEADERS += \
    joint.h \
    OmniWheelVariables.h \
    BasicMath.h \
    BasicMatrix.h \
    BasicTrajectory.h \
    taskGeneral.h \
    taskmotion.h \
    RBLog.h \
    ManualCAN.h

