#ifndef PODOSERVER_H
#define PODOSERVER_H

#include <LAN/RBTCPServer.h>
#include <QDataStream>
#include "LANData/RBLANData.h"
#include "LANData/ROSLANData.h"


class PODO_GUI_Server : public RBTCPServer
{
    Q_OBJECT
public:
    PODO_GUI_Server();

    QByteArrays dataReceived;
    QByteArray  RBData;

protected slots:
    virtual void    RBReadData();

private:
    int         dataSize;
};


class PODO_ROS_Server : public RBTCPServer
{
    Q_OBJECT
public:
    PODO_ROS_Server();

    QByteArrays dataReceived;
    QByteArray  RBData;

protected slots:
    virtual void    RBReadData();

private:
    int         dataSize;
};

#endif // PODOSERVER_H
