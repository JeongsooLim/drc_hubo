TEMPLATE = app
CONFIG  += console
CONFIG  -= app_bundle



INCLUDEPATH += \
    ../../SHARE/Headers

LIBS    += \
    -lpthread \
    -lrt \


SOURCES += main.cpp

HEADERS += \
    BasicFiles/BasicSetting.h \
    BasicFiles/BasicJoint.h


