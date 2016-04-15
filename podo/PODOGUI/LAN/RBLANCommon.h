#ifndef RBLANCOMMON_H
#define RBLANCOMMON_H

#include <iostream>
#include <unistd.h>
#include <QByteArray>
#include <QVector>
#include <QString>
#include <QTimer>

#include "RBLog.h"

#define     DATA_HEADER         ((int)(0xf0f0f0f0))
#define     DATA_TAIL           ((int)(0xFF00FF00))

enum{RBLAN_SUCCESS = 0, RBLAN_FAIL};
enum{RBLAN_CS_CONNECTED = 0, RBLAN_CS_DISCONNECTED};
enum{STATE_HEADER = 0, STATE_TYPE, STATE_SIZE, STATE_DATA, STATE_TAIL};



typedef union{
    char    cData[4];
    int     iData;
} uniInt;

typedef union{
    char    cData[2];
    short   sData;
} uniShort;

typedef union{
    char    cData[4];
    float   fData;
} uniFloat;


typedef struct{
    int         type;
    QByteArray  data;
} NORMAL_DATA;


typedef QVector<QByteArray>     QByteArrays;
typedef QVector<NORMAL_DATA>    NORMAL_DATAS;


inline void AppendFloat(QByteArray &ba, float data){
    char temp[4];
    memcpy(temp, &data, 4);
    ba.append(temp, 4);
}

inline void AppendInt(QByteArray &ba, int data){
    char temp[4];
    memcpy(temp, &data, 4);
    ba.append(temp, 4);
}

#endif // RBLANCOMMON_H
