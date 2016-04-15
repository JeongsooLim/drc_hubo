#ifndef RBTCPCLIENT_H
#define RBTCPCLIENT_H

#include <QTcpSocket>
#include "RBLANCommon.h"



class RBTCPClient : public QTcpSocket
{
    Q_OBJECT
public:
    RBTCPClient(QObject *parent = 0);
    ~RBTCPClient();

    void RBConnect(QString _ip = "127.0.0.1", quint16 _port = 4000);
    void RBDisconnect();
    void RBSendData(QByteArray &data);

    quint8          RBConnectionState;

private slots:
    void            RBClientConnected();
    void            RBClientDisconnected();

protected slots:
    virtual void    RBReadData() = 0;

protected:
    QTcpSocket			*RBClientSocket;
    QString             PortName;

private:
    QString				RBIpAdress;
    quint16				RBPortNumber;
};

#endif // RBTCPCLIENT_H
