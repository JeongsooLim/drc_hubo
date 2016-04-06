#include "CoreThread.h"


extern pRBCORE_SHM_COMMAND     sharedCMD;
extern pRBCORE_SHM_REFERENCE   sharedREF;
extern pRBCORE_SHM_SENSOR      sharedSEN;
extern pUSER_SHM               sharedUSER;

CoreThread::CoreThread()
{
}

void CoreThread::run(){
    QTimer timerPODO2GUI, timerGUI2PODO;
    CoreWorker worker;

    connect(&timerPODO2GUI, SIGNAL(timeout()), &worker, SLOT(onPODO2GUI()));
    connect(&timerGUI2PODO, SIGNAL(timeout()), &worker, SLOT(onGUI2PODO()));

    timerPODO2GUI.start(50);
    timerGUI2PODO.start(50);
    exec();
}



CoreWorker::CoreWorker(){
    serverPODOGUI = new PODO_GUI_Server();
    serverPODOGUI->RBServerOpen(QHostAddress::AnyIPv4, 4000);
}


void CoreWorker::SendtoGUI(){
    memcpy(&(DATA_PODO.CoreREF), sharedREF, sizeof(RBCORE_SHM_REFERENCE));
    memcpy(&(DATA_PODO.CoreSEN), sharedSEN, sizeof(RBCORE_SHM_SENSOR));
    memcpy(&(DATA_PODO.CoreCMD), sharedCMD, sizeof(RBCORE_SHM_COMMAND));

    memcpy(&(DATA_PODO.UserM2G), &(sharedUSER->M2G), sizeof(MOTION2GUI));

    QByteArray SendData = QByteArray::fromRawData((char*)&DATA_PODO, sizeof(LAN_PODO2GUI));
    serverPODOGUI->RBSendData(SendData);
}

void CoreWorker::ReadfromGUI(){
    QByteArray tempData = serverPODOGUI->dataReceived[0];
    serverPODOGUI->dataReceived.pop_front();

//    USER_COMMAND cmd;
//    memcpy(&cmd, tempData, sizeof(USER_COMMAND));

//    int target = cmd.COMMAND_TARGET;
//    for(int i=0; i<MAX_COMMAND_DATA; i++){
//        sharedCMD->COMMAND[target].USER_PARA_CHAR[i]    = cmd.COMMAND_DATA.USER_PARA_CHAR[i];
//        sharedCMD->COMMAND[target].USER_PARA_INT[i]     = cmd.COMMAND_DATA.USER_PARA_INT[i];
//        sharedCMD->COMMAND[target].USER_PARA_FLOAT[i]   = cmd.COMMAND_DATA.USER_PARA_FLOAT[i];
//        sharedCMD->COMMAND[target].USER_PARA_DOUBLE[i]  = cmd.COMMAND_DATA.USER_PARA_DOUBLE[i];
//    }
//    sharedCMD->COMMAND[target].USER_COMMAND = cmd.COMMAND_DATA.USER_COMMAND;

    LAN_GUI2PODO tempDATA;
    memcpy(&tempDATA, tempData, sizeof(LAN_GUI2PODO));
    memcpy(&(sharedUSER->G2M), &(tempDATA.UserG2M), sizeof(GUI2MOTION));

    int target = tempDATA.UserCMD.COMMAND_TARGET;
    for(int i=0; i<MAX_COMMAND_DATA; i++){
        sharedCMD->COMMAND[target].USER_PARA_CHAR[i]    = tempDATA.UserCMD.COMMAND_DATA.USER_PARA_CHAR[i];
        sharedCMD->COMMAND[target].USER_PARA_INT[i]     = tempDATA.UserCMD.COMMAND_DATA.USER_PARA_INT[i];
        sharedCMD->COMMAND[target].USER_PARA_FLOAT[i]   = tempDATA.UserCMD.COMMAND_DATA.USER_PARA_FLOAT[i];
        sharedCMD->COMMAND[target].USER_PARA_DOUBLE[i]  = tempDATA.UserCMD.COMMAND_DATA.USER_PARA_DOUBLE[i];
    }
    sharedCMD->COMMAND[target].USER_COMMAND = tempDATA.UserCMD.COMMAND_DATA.USER_COMMAND;
}

void CoreWorker::onPODO2GUI(){
    if(serverPODOGUI->RBConnectionState == RBLAN_CS_CONNECTED){
        SendtoGUI();
    }
}

void CoreWorker::onGUI2PODO(){
    if(serverPODOGUI->dataReceived.size() > 0){
        ReadfromGUI();
    }
}

