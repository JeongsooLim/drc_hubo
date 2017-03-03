#ifndef CORETHREAD_H
#define CORETHREAD_H

#include <sys/mman.h>
#include <fcntl.h>

#include <QThread>
#include <QByteArray>
#include <QTimer>

#include "PODOServer.h"


class CoreThread : public QThread
{
    Q_OBJECT
public:
    CoreThread();

protected:
    void run();
};


class CoreWorker : public QObject
{
    Q_OBJECT
public:
    CoreWorker();

private slots:
    void onPODO2GUI();
    void onGUI2PODO();

private:
    PODO_GUI_Server         *serverPODOGUI;

    LAN_PODO2GUI            DATA_PODO;

    void    SendtoGUI();
    void    ReadfromGUI();

};



#endif // CORETHREAD_H
