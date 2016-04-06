#ifndef LANDIALOG_H
#define LANDIALOG_H

#include <QDialog>
#include <QTimer>
#include <QSettings>
#include <LAN/RBTCPClient.h>


#include "../SHARE/Headers/LANData/RBLANData.h"


namespace Ui {
class LANDialog;
}

class PODOClient;

class LANDialog : public QDialog
{
    Q_OBJECT

public:
    explicit LANDialog(QWidget *parent = 0);
    ~LANDialog();


    void SendCommand(USER_COMMAND &cmd);

    PODOClient      *clientPODO;
    GUI2MOTION      *G2MData;

protected:
    virtual void hideEvent(QHideEvent *){emit SIG_LAN_HIDE();}

signals:
    void NewPODOData();
    void SIG_LAN_HIDE();

    void SIG_LAN_ON_OFF(bool onoff);


private slots:
    void ReadDataThread();
    void on_BTN_LAN_CONNECT_clicked();

    void LAN_Connected();
    void LAN_Disconnected();

private:
    Ui::LANDialog *ui;
    QTimer  *readTimer;

    QString     settingFile;
};


class PODOClient : public RBTCPClient
{
    Q_OBJECT
public:
    PODOClient();

    QByteArrays     dataReceived;

protected slots:
    virtual void    RBReadData();

private:
    int     dataSize;
};

#endif // LANDIALOG_H
