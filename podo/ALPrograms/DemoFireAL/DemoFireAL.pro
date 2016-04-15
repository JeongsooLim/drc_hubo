
QT       += core
QT       -= gui

TARGET = DemoFireAL
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
   -L../../SHARE/Libs       -lKINE_DRC_HUBO3 \
   -L../../SHARE/Libs	-lik_math3

SOURCES += main.cpp \
	joint.cpp \
	taskmotion.cpp \
	BasicTrajectory.cpp \
    DebrisMotion.cpp \
    DebrisMotionScript.cpp



HEADERS += \
	joint.h \
	taskmotion.h \
	taskGeneral.h \
	BasicTrajectory.h \
	BasicMatrix.h \
	BasicMath.h \
	RBLog.h \
    DebrisMotion.h
