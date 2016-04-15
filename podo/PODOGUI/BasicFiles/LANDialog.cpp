#include "LANDialog.h"
#include "ui_LANDialog.h"


extern LAN_PODO2GUI     PODO_DATA;


LANDialog::LANDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::LANDialog)
{
    ui->setupUi(this);
    settingFile = QApplication::applicationDirPath().left(1) + "/demosettings.ini";
    settingFile = "configs/config.ini";
    QSettings settings(settingFile, QSettings::NativeFormat);
    ui->LE_IP_ADDR->setText(settings.value("ip", "").toString());
    ui->LE_PORT_NUM->setText(settings.value("port", "").toString());

    G2MData = (pGUI2MOTION)malloc(sizeof(GUI2MOTION));
    clientPODO = new PODOClient();
    connect(clientPODO, SIGNAL(SIG_Connected()), this, SLOT(LAN_Connected()));
    connect(clientPODO, SIGNAL(SIG_Disconnected()), this, SLOT(LAN_Disconnected()));

    readTimer = new QTimer(this);
    connect(readTimer, SIGNAL(timeout()), this, SLOT(ReadDataThread()));
    readTimer->start(20);
}

LANDialog::~LANDialog()
{
    delete ui;
}

void LANDialog::ReadDataThread(){
    if(clientPODO->dataReceived.size() > 0){
        QByteArray tempPODOData = clientPODO->dataReceived[0];
        clientPODO->dataReceived.pop_front();

        memcpy(&PODO_DATA, tempPODOData, sizeof(LAN_PODO2GUI));
        emit NewPODOData();
    }
}

void LANDialog::SendCommand(USER_COMMAND &cmd){
    //QByteArray tempSendData = QByteArray::fromRawData((char *)&cmd, sizeof(USER_COMMAND));

    LAN_GUI2PODO tempDATA;
    memcpy(&(tempDATA.UserCMD), &cmd, sizeof(USER_COMMAND));
    memcpy(&(tempDATA.UserG2M), G2MData, sizeof(GUI2MOTION));
    QByteArray tempSendData = QByteArray::fromRawData((char *)&tempDATA, sizeof(LAN_GUI2PODO));
    G2MData->ros_cmd.cmd = 0;

    clientPODO->RBSendData(tempSendData);
}

void LANDialog::on_BTN_LAN_CONNECT_clicked(){
    if(clientPODO->RBConnectionState == RBLAN_CS_DISCONNECTED){
        clientPODO->RBConnect(ui->LE_IP_ADDR->text(), ui->LE_PORT_NUM->text().toInt());
    }else{
        clientPODO->RBDisconnect();
    }
}

void LANDialog::LAN_Connected(){
    ui->BTN_LAN_CONNECT->setText("Disconnect");
    QSettings settings(settingFile, QSettings::NativeFormat);
    settings.setValue("ip", ui->LE_IP_ADDR->text());
    settings.setValue("port", ui->LE_PORT_NUM->text());

    emit SIG_LAN_ON_OFF(true);
}
void LANDialog::LAN_Disconnected(){
    ui->BTN_LAN_CONNECT->setText("Connect");

    emit SIG_LAN_ON_OFF(false);
}


//=====================================
//=====================================


PODOClient::PODOClient(){
    dataSize = sizeof(LAN_PODO2GUI);
    dataReceived.clear();
    PortName = "PODOClient";
}

void PODOClient::RBReadData(){
    QDataStream in(RBClientSocket);
    in.setVersion(QDataStream::Qt_5_2);

    if(RBClientSocket->bytesAvailable() < dataSize)
        return;

    char *temp = new char[dataSize];
    while(RBClientSocket->bytesAvailable() >= dataSize){
        in.readRawData(temp, dataSize);
        QByteArray data;
        data.clear();
        data.append(temp, dataSize);
        dataReceived.push_back(data);
    }
    delete []temp;
}

