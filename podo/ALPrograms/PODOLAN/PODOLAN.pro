#-------------------------------------------------
#
# Project created by QtCreator 2016-02-17T08:41:59
#
#-------------------------------------------------

QT       += core network
QT       -= gui

TARGET = PODOLAN
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

INCLUDEPATH += \
    ../../SHARE/Headers

LIBS += -lrt

SOURCES += \
    main.cpp \
    LAN/RBTCPClient.cpp \
    LAN/RBTCPServer.cpp \
    CoreThread.cpp \
    PODOServer.cpp \
    ROSThread.cpp


HEADERS += \
    LAN/RBLANCommon.h \
    LAN/RBLog.h \
    LAN/RBTCPClient.h \
    LAN/RBTCPServer.h \
    CoreThread.h \
    PODOServer.h \
    ROSThread.h

