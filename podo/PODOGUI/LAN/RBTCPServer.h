#ifndef RBTCPSERVER_H
#define RBTCPSERVER_H

#include <QTcpServer>
#include <QTcpSocket>
#include "RBLANCommon.h"

class QTcpServer;

class RBTCPServer : public QTcpServer
{
    Q_OBJECT
public:
    RBTCPServer(QObject *parent = 0);
    ~RBTCPServer();

    void RBServerOpen(QHostAddress::SpecialAddress _host = QHostAddress::Any, quint16 _port = 4000);
    void RBSendData(QByteArray &data);

    quint8      RBConnectionState;

private slots:
    void            RBNewConnection();
    void            RBClientDisconnected();

protected slots:
    virtual void    RBReadData() = 0;

signals:
    void    SIG_NewConnection();
    void    SIG_DisConnected();

protected:
    QTcpServer                      *RBTcpServer;
    QTcpSocket                      *RBTcpClient;

private:
    QHostAddress::SpecialAddress    RBHostAddress;
    quint16                         RBPortNumber;

};


#endif // RBTCPSERVER_H
