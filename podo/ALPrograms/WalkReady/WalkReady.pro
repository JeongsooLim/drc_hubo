#-------------------------------------------------
#
# Project created by QtCreator 2014-02-11T10:41:29
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = WalkReady
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
   #-L../../SHARE/Libs	-lik_math2 \
   -L../../SHARE/Libs       -lKINE_DRC_HUBO4 \
   -L../../SHARE/Libs	-lik_math4 \
#                -L../../SHARE/Libs	-lKINE_DRC_HUBO_TJ \
#               -L../../SHARE/Libs	-lik_math2 \


SOURCES += main.cpp \
    joint.cpp \
    ManualCAN.cpp

HEADERS += \
    joint.h \
    RBLog.h \
    kine_drc_hubo_tj.h \
    ManualCAN.h

