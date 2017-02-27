#include "ROSThread.h"


extern pRBCORE_SHM_COMMAND     sharedCMD;
extern pRBCORE_SHM_REFERENCE   sharedREF;
extern pRBCORE_SHM_SENSOR      sharedSEN;
extern pUSER_SHM               sharedUSER;

extern int     __IS_GAZEBO;


ROSThread::ROSThread()
{
}

void ROSThread::run(){

    ROSWorker worker;
    QTimer fastTimer;
    QTimer timerPODO2ROS, timerROS2PODO;

    if(__IS_GAZEBO){
        worker.timePeriod = 0.05; // 50ms period for sending
        worker.timePrev = worker.timeCur = sharedSEN->Sim_Time_sec + sharedSEN->Sim_Time_nsec/1000000000.0;

        connect(&fastTimer, SIGNAL(timeout()), &worker, SLOT(onFastTimer()));
        fastTimer.start(1);
    }else{
        connect(&timerPODO2ROS, SIGNAL(timeout()), &worker, SLOT(onPODO2ROS()));
        connect(&timerROS2PODO, SIGNAL(timeout()), &worker, SLOT(onROS2PODO()));

        timerPODO2ROS.start(50);
        timerROS2PODO.start(50);
    }
    exec();
}



ROSWorker::ROSWorker(){
    serverPODOROS = new PODO_ROS_Server();
    serverPODOROS->RBServerOpen(QHostAddress::AnyIPv4, 5000);
    if(__IS_GAZEBO){
        connect(serverPODOROS, SIGNAL(SIG_NewConnection()), this, SLOT(onNewConnection()));
    }
}

void ROSWorker::onNewConnection(){
    timePrev = timeCur = sharedSEN->Sim_Time_sec + sharedSEN->Sim_Time_nsec/1000000000.0;
}


void ROSWorker::onFastTimer(){
    // read data from ROS
    onROS2PODO();

    // send data to ROS
    timeCur = sharedSEN->Sim_Time_sec + sharedSEN->Sim_Time_nsec/1000000000.0;
    if(timeCur - timePrev >= timePeriod){
        timePrev = timeCur;
        onPODO2ROS();
    }else if(timeCur < timePrev-0.001){
        FILE_LOG(logERROR) << "SimTime Reversed..";
        FILE_LOG(logERROR) << "Probably New Gazebo is turned on..";
        FILE_LOG(logERROR) << "Reset the local timer..";

        timePrev = timeCur = sharedSEN->Sim_Time_sec + sharedSEN->Sim_Time_nsec/1000000000.0;
    }
}


void ROSWorker::SendtoROS(){
    for(int i=0; i<NO_OF_JOINTS; i++){
        TXData.JointReference[i] = sharedSEN->ENCODER[MC_GetID(i)][MC_GetCH(i)].CurrentReference;
        TXData.JointEncoder[i] = sharedSEN->ENCODER[MC_GetID(i)][MC_GetCH(i)].CurrentPosition;
    }
    TXData.odom_x = sharedUSER->odom_data[0];
    TXData.odom_y = sharedUSER->odom_data[1];
    TXData.odom_theta = sharedUSER->odom_data[2];
    TXData.vx = sharedUSER->odom_data[3];
    TXData.vy = sharedUSER->odom_data[4];
    TXData.vth = sharedUSER->odom_data[5];

    memcpy(&(TXData.cmd), &(sharedUSER->G2M.ros_cmd), sizeof(LAN_GENERAL_COMMAND));
    sharedUSER->G2M.ros_cmd.cmd = 0;

    //FILE_LOG(logWARNING) << "Send to ROS: " << sizeof(LAN_PODO2ROS);
    QByteArray SendData = QByteArray::fromRawData((char*)&TXData, sizeof(LAN_PODO2ROS));
    serverPODOROS->RBSendData(SendData);
}

void ROSWorker::ReadfromROS(){
    QByteArray tempData = serverPODOROS->dataReceived[0];
    serverPODOROS->dataReceived.pop_front();

    memcpy(&RXData, tempData.data(), sizeof(RXData));

    sharedUSER->vel_cmd[0] = RXData.vx;
    sharedUSER->vel_cmd[1] = RXData.vth;
    sharedUSER->M2G.obj_pos[0] = RXData.pos[0];
    sharedUSER->M2G.obj_pos[1] = RXData.pos[1];
    sharedUSER->M2G.obj_pos[2] = RXData.pos[2];
}


void ROSWorker::onPODO2ROS(){
    if(serverPODOROS->RBConnectionState == RBLAN_CS_CONNECTED){
        SendtoROS();
    }
}

void ROSWorker::onROS2PODO(){
    if(serverPODOROS->dataReceived.size() > 0){
        ReadfromROS();
    }
}

