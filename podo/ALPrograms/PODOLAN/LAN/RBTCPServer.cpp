#include "RBTCPServer.h"

RBTCPServer::RBTCPServer(QObject *parent)
    : QTcpServer(parent), RBTcpServer(0)
{
    RBConnectionState = RBLAN_CS_DISCONNECTED;
    RBTcpClient = new QTcpSocket();
}
RBTCPServer::~RBTCPServer()
{
}

void RBTCPServer::RBServerOpen(QHostAddress::SpecialAddress _host, quint16 _port){
    RBHostAddress = _host;
    RBPortNumber = _port;

    RBTcpServer = new QTcpServer(this);
    connect(RBTcpServer, SIGNAL(newConnection()), this, SLOT(RBNewConnection()));

    if(!RBTcpServer->listen(RBHostAddress, RBPortNumber)){
        FILE_LOG(logERROR) << "RBServer unable to start the server";
    }else{
        FILE_LOG(logSUCCESS) << "RBServer open successed with port(" << RBTcpServer->serverPort() << ")";
    }
}

void RBTCPServer::RBNewConnection()
{
    if(RBConnectionState == RBLAN_CS_DISCONNECTED){
        RBTcpClient = RBTcpServer->nextPendingConnection();
        connect(RBTcpClient, SIGNAL(disconnected()), this, SLOT(RBClientDisconnected()));
        connect(RBTcpClient, SIGNAL(readyRead()), this, SLOT(RBReadData()));
        RBConnectionState = RBLAN_CS_CONNECTED;
        FILE_LOG(logINFO) << "New client is connected";

        // emit signal
        emit SIG_NewConnection();
    }else{
        QTcpSocket *dummySocket = RBTcpServer->nextPendingConnection();
        dummySocket->close();
        FILE_LOG(logWARNING) << "Client is already connected";
    }
}

void RBTCPServer::RBClientDisconnected(){
    RBTcpClient->deleteLater();
    RBConnectionState = RBLAN_CS_DISCONNECTED;
    FILE_LOG(logINFO) << "Client is disconnected";

    // emit signal
    emit SIG_DisConnected();
}

void RBTCPServer::RBSendData(QByteArray &data){
    RBTcpClient->write(data);
}
