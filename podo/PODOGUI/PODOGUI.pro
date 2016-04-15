#-------------------------------------------------
#
# Project created by QtCreator 2016-02-17T10:05:39
#
#-------------------------------------------------

QT       += core gui sql network opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = PODOGUI
TEMPLATE = app


###
INCLUDEPATH += \
    ../SHARE/Headers/RBModel

LIBS += \
    -L../SHARE/Libs/ -lRBModel
###


SOURCES += \
    main.cpp\
    GUIMainWindow.cpp \
    LAN/RBTCPClient.cpp \
    LAN/RBTCPServer.cpp \
    BasicFiles/RBDataBase.cpp \
    BasicFiles/LANDialog.cpp \
    BasicFiles/PODOALDialog.cpp \
    BasicFiles/JointDialog.cpp \
    BasicFiles/SensorDialog.cpp \
    BasicFiles/ModelDialog.cpp \
    BasicFiles/SettingDialog.cpp \
    TutorialDialog.cpp

HEADERS  += \
    GUIMainWindow.h \
    CommonHeader.h \
    LAN/RBLANCommon.h \
    LAN/RBLog.h \
    LAN/RBTCPClient.h \
    LAN/RBTCPServer.h \
    BasicFiles/RBDataBase.h \
    BasicFiles/RBDataType.h \
    BasicFiles/RBLog.h \
    BasicFiles/LANDialog.h \
    BasicFiles/PODOALDialog.h \
    BasicFiles/JointDialog.h \
    BasicFiles/SensorDialog.h \
    BasicFiles/ModelDialog.h \
    BasicFiles/SettingDialog.h \
    TutorialDialog.h


FORMS    += \
    GUIMainWindow.ui \
    BasicFiles/LANDialog.ui \
    BasicFiles/PODOALDialog.ui \
    BasicFiles/JointDialog.ui \
    BasicFiles/SensorDialog.ui \
    BasicFiles/ModelDialog.ui \
    BasicFiles/SettingDialog.ui \
    TutorialDialog.ui
