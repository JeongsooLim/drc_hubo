#include "RBTCPClient.h"

RBTCPClient::RBTCPClient(QObject *parent)
    : QTcpSocket(parent)
{
    RBConnectionState = RBLAN_CS_DISCONNECTED;
    RBClientSocket = new QTcpSocket(this);

    connect(RBClientSocket, SIGNAL(connected()), this, SLOT(RBClientConnected()));
    connect(RBClientSocket, SIGNAL(disconnected()), this, SLOT(RBClientDisconnected()));
}
RBTCPClient::~RBTCPClient()
{
}

void RBTCPClient::RBClientConnected(){
    RBConnectionState = RBLAN_CS_CONNECTED;
    FILE_LOG(logINFO) << "Client is connected..!! (" << PortName.toStdString().data() << ")";
    connect(RBClientSocket, SIGNAL(readyRead()), this, SLOT(RBReadData()));
    emit SIG_Connected();
}

void RBTCPClient::RBClientDisconnected(){
    RBConnectionState = RBLAN_CS_DISCONNECTED;
    FILE_LOG(logINFO) << "Client is disconnected..!! (" << PortName.toStdString().data() << ")";
    emit SIG_Disconnected();
}

void RBTCPClient::RBConnect(QString _ip, quint16 _port){
    RBIpAdress = _ip;
    RBPortNumber = _port;
    RBClientSocket->connectToHost(_ip, _port);
}

void RBTCPClient::RBDisconnect(void){    
    RBClientSocket->close();
}

void RBTCPClient::RBSendData(QByteArray &data){
    RBClientSocket->write(data);
}

