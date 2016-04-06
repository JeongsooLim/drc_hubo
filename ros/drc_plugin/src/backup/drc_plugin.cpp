
#include "drc_plugin.h"


#define PODO_ADDR       "10.12.3.30"
#define PODO_PORT       8888

const float     D2Rf = 0.0174533;
const float     R2Df = 57.2957802;

namespace gazebo
{

void DRCPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    std::cout << "=================================" << std::endl;
    std::cout << "   Now Start the DRCPlugin..!!" << std::endl;
    std::cout << "=================================" << std::endl;
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized()){
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }
    ros::NodeHandle n;
    //joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);

    filt_alpha = 0.05;
    new_ref_cnt = 0;
    home_ref_RWH = home_ref_LWH = 0.0;

    model = _model;
    JCon = new physics::JointController(model);

    std::string modelName = model->GetName();
    std::cout << "Model Name : " << modelName << std::endl;

    // Create Socket ---------------------
    char ip[20];
    FILE *fpNet = NULL;
    fpNet = fopen("/home/gazebo/DRC_HUBO_GAZEBO_PLUGIN/build/network.txt", "r");
    if(fpNet == NULL){
        std::cout << ">>> Network File Open Error..!!" << std::endl;
        sprintf(ip, PODO_ADDR);
    }else{
        std::cout << ">>> Network File Open Success..!!" << std::endl;
        fscanf(fpNet, "%s", ip);
        fclose(fpNet);
    }

    if(CreateSocket(ip, PODO_PORT)){
        std::cout << "Created Socket.." << std::endl;
        RXDataSize = sizeof(DRC_GAZEBO_JOINT);
        TXDataSize = sizeof(DRC_GAZEBO_SENSOR);
        RXBuffer = (void*)malloc(RXDataSize);
        TXBuffer = (void*)malloc(TXDataSize);
        int threadID = pthread_create(&LANTHREAD_t, NULL, &LANThread, this);
        if(threadID < 0){
            std::cout << "Create Thread Error.." << std::endl;
        }
    }else{
        std::cout << "Create Socket Error.." << std::endl;
    }

    // Load Gains for Joints --------------
    FILE *fpGain = NULL;
    fpGain = fopen("/home/gazebo/DRC_HUBO_GAZEBO_PLUGIN/build/gain.txt", "r");
    if(fpGain == NULL){
        std::cout << ">>> Gain File Open Error..!!" << std::endl;
    }else{
        std::cout << ">>> Gain File Open Success..!!" << std::endl;
        for(int i=0; i<NO_OF_JOINTS; i++){
            fscanf(fpGain, "%f, %f, %f\n", &PIDGains[i][0], &PIDGains[i][1], &PIDGains[i][2]);
        }
        fclose(fpGain);
    }

    joint_state.name.resize(NO_OF_JOINTS+9*2+4+1);
    joint_state.position.resize(NO_OF_JOINTS+9*2+4+1);

    for(int i=0; i<NO_OF_JOINTS; i++){
        if(i == RHAND || i == LHAND)
            continue;
        JPtrs[i] = model->GetJoint(JointNameList[i]);
        JCon->AddJoint(JPtrs[i]);
        JCon->SetPositionPID(JPtrs[i]->GetScopedName(),common::PID(PIDGains[i][0],PIDGains[i][1],PIDGains[i][2]));
        GainOverride[i].current = 100;
        joint_state.name[i] = JPtrs[i]->GetName();
    }
    joint_state.name[RHAND] = "RFWH"; // use for foot passive wheel
    joint_state.name[LHAND] = "LFWH"; // use for foot passive wheel

    JPtr_LHAND[0] = model->GetJoint("LHAND_a1");
    JPtr_LHAND[1] = model->GetJoint("LHAND_a2");
    JPtr_LHAND[2] = model->GetJoint("LHAND_a3");
    JPtr_LHAND[3] = model->GetJoint("LHAND_b1");
    JPtr_LHAND[4] = model->GetJoint("LHAND_b2");
    JPtr_LHAND[5] = model->GetJoint("LHAND_b3");
    JPtr_LHAND[6] = model->GetJoint("LHAND_c1");
    JPtr_LHAND[7] = model->GetJoint("LHAND_c2");
    JPtr_LHAND[8] = model->GetJoint("LHAND_c3");

    JPtr_RHAND[0] = model->GetJoint("RHAND_a1");
    JPtr_RHAND[1] = model->GetJoint("RHAND_a2");
    JPtr_RHAND[2] = model->GetJoint("RHAND_a3");
    JPtr_RHAND[3] = model->GetJoint("RHAND_b1");
    JPtr_RHAND[4] = model->GetJoint("RHAND_b2");
    JPtr_RHAND[5] = model->GetJoint("RHAND_b3");
    JPtr_RHAND[6] = model->GetJoint("RHAND_c1");
    JPtr_RHAND[7] = model->GetJoint("RHAND_c2");
    JPtr_RHAND[8] = model->GetJoint("RHAND_c3");
    for(int i=0; i<9; i++){
        JCon->AddJoint(JPtr_RHAND[i]);
        JCon->SetPositionPID(JPtr_RHAND[i]->GetScopedName(),common::PID(20,0,0.01));
        JCon->AddJoint(JPtr_LHAND[i]);
        JCon->SetPositionPID(JPtr_LHAND[i]->GetScopedName(),common::PID(20,0,0.01));
    }
    for(int i=0; i<9; i++){
        joint_state.name[NO_OF_JOINTS + i] = JPtr_LHAND[i]->GetName();
        joint_state.name[NO_OF_JOINTS + 9 + i] = JPtr_RHAND[i]->GetName();
    }

    JPtr_RAFT = model->GetJoint("RAFT");
    JPtr_LAFT = model->GetJoint("LAFT");
    JPtr_RWFT = model->GetJoint("RWFT");
    JPtr_LWFT = model->GetJoint("LWFT");
    JCon->AddJoint(JPtr_RAFT);
    JCon->AddJoint(JPtr_LAFT);
    JCon->AddJoint(JPtr_RWFT);
    JCon->AddJoint(JPtr_LWFT);
    JCon->SetPositionPID(JPtr_RAFT->GetScopedName(),common::PID(PIDGains[RAR][0],PIDGains[RAR][1],PIDGains[RAR][2]));
    JCon->SetPositionPID(JPtr_LAFT->GetScopedName(),common::PID(PIDGains[LAR][0],PIDGains[LAR][1],PIDGains[LAR][2]));
    JCon->SetPositionPID(JPtr_RWFT->GetScopedName(),common::PID(0,0,0.001));
    JCon->SetPositionPID(JPtr_LWFT->GetScopedName(),common::PID(0,0,0.001));

    joint_state.name[NO_OF_JOINTS + 18 + 0] = JPtr_RAFT->GetName();
    joint_state.name[NO_OF_JOINTS + 18 + 1] = JPtr_LAFT->GetName();
    joint_state.name[NO_OF_JOINTS + 18 + 2] = JPtr_RWFT->GetName();
    joint_state.name[NO_OF_JOINTS + 18 + 3] = JPtr_LWFT->GetName();


//    JPtr_Head = model->GetJoint("head_joint");
//    JCon->AddJoint(JPtr_Head);
//    JCon->SetPositionPID(JPtr_Head->GetScopedName(), common::PID(1,0,0.01));
//    joint_state.name[NO_OF_JOINTS + 18 + 4] = JPtr_Head->GetName();

    // provide feedback for getting wrench---
    for(int i=0; i<NO_OF_JOINTS; i++){
        if(i == RHAND || i == LHAND)
            continue;
        JPtrs[i]->SetProvideFeedback(true);
    }

    JPtr_RAFT->SetProvideFeedback(true);
    JPtr_LAFT->SetProvideFeedback(true);
    JPtr_RWFT->SetProvideFeedback(true);
    JPtr_LWFT->SetProvideFeedback(true);
    // ---------------------------------------


    if(connectionStatus){
        for(int i=0;i<=NO_OF_JOINTS;i++){
            refs[i] = RXJointData.JointReference[i]*D2Rf;
        }
    }else{
        for(int i=0;i<=NO_OF_JOINTS;i++){
            refs[i] = 0.0;
        }
        ref_RHAND = ref_LHAND = 1.4708;
    }
    refs[LEB] += -20*D2Rf;
    refs[REB] += -20*D2Rf;
    refs[RSR] += -15*D2Rf;
    refs[LSR] += 15*D2Rf;


    for(int i=0; i<=NO_OF_JOINTS; i++){
        if(i == RHAND || i == LHAND)
            continue;
        JCon->SetJointPosition(JPtrs[i],refs[i]);
    }
    JCon->SetJointPosition(JPtr_RAFT,0);
    JCon->SetJointPosition(JPtr_LAFT,0);
    JCon->SetJointPosition(JPtr_RWFT,0);
    JCon->SetJointPosition(JPtr_LWFT,0);

    RAFT = boost::dynamic_pointer_cast<sensors::ForceTorqueSensor>(sensors::get_sensor("FT_RAFT"));
    LAFT = boost::dynamic_pointer_cast<sensors::ForceTorqueSensor>(sensors::get_sensor("FT_LAFT"));
    IMU = boost::dynamic_pointer_cast<sensors::ImuSensor>(sensors::get_sensor("imu"));

    if(!LAFT || !RAFT || !IMU)
        std::cout << "SENSOR ERROR (NULL)" << std::endl;

    IMU->SetActive(true);
    LAFT->SetActive(true);
    RAFT->SetActive(true);


    IMU->SetUpdateRate(1000);
    LAFT->SetUpdateRate(1000);
    RAFT->SetUpdateRate(1000);

    // Listen to the update event. This event is broadcast every simulation iteration.
    UpdateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&DRCPlugin::OnUpdate, this, _1));
    LAFTupdateConnection = LAFT->ConnectUpdated(boost::bind(&DRCPlugin::OnLAFTUpdate, this));
    RAFTupdateConnection = RAFT->ConnectUpdated(boost::bind(&DRCPlugin::OnRAFTUpdate, this));
    IMUupdateConnection  = IMU->ConnectUpdated(boost::bind(&DRCPlugin::OnIMUUpdate, this));

}

void DRCPlugin::OnLAFTUpdate(){
    math::Vector3 LAF = LAFT->GetForce();
    math::Vector3 LAT = LAFT->GetTorque();
    filt_LAF = filt_alpha*LAF + (1-filt_alpha)*filt_LAF;
    filt_LAT = filt_alpha*LAT + (1-filt_alpha)*filt_LAT;
    if(connectionStatus){
        TXSensorData.FTSensor[1].force[0]   = filt_LAF.x;
        TXSensorData.FTSensor[1].force[1]   = filt_LAF.y;
        TXSensorData.FTSensor[1].force[2]   = filt_LAF.z;
        TXSensorData.FTSensor[1].torque[0]  = filt_LAT.x;
        TXSensorData.FTSensor[1].torque[1]  = filt_LAT.y;
        TXSensorData.FTSensor[1].torque[2]  = filt_LAT.z;
    }
}

void DRCPlugin::OnRAFTUpdate(){
    math::Vector3 RAF = RAFT->GetForce();
    math::Vector3 RAT = RAFT->GetTorque();
    filt_RAF = filt_alpha*RAF + (1-filt_alpha)*filt_RAF;
    filt_RAT = filt_alpha*RAT + (1-filt_alpha)*filt_RAT;
    if(connectionStatus){
        TXSensorData.FTSensor[0].force[0]   = filt_RAF.x;
        TXSensorData.FTSensor[0].force[1]   = filt_RAF.y;
        TXSensorData.FTSensor[0].force[2]   = filt_RAF.z;
        TXSensorData.FTSensor[0].torque[0]  = filt_RAT.x;
        TXSensorData.FTSensor[0].torque[1]  = filt_RAT.y;
        TXSensorData.FTSensor[0].torque[2]  = filt_RAT.z;
    }
}

void DRCPlugin::OnIMUUpdate(){
    math::Vector3 AV = IMU->GetAngularVelocity();
    math::Vector3 LA = IMU->GetLinearAcceleration();
    math::Quaternion q = IMU->GetOrientation();
    if(connectionStatus){
        float l = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
        float w, x, y, z;
        if(l > 0.00001){
            l = sqrt(l);
            w = q.w/l; x = q.x/l; y = q.y/l; z = q.z/l;
        }else{
            w = 1; x = 0; y = 0; z = 0;
        }
        // roll, pitch, yaw
        TXSensorData.IMUSensor[0] = atan2(2*(w*x + y*z), 1-2*(x*x + y*y))*R2Df;
        TXSensorData.IMUSensor[1] = asin(2*(w*y - z*x))*R2Df;
        TXSensorData.IMUSensor[2] = atan2(2*(w*z + x*y), 1-2*(y*y + z*z))*R2Df;
        // rvel, pvel, yvel
        TXSensorData.IMUSensor[3] = AV.x*R2Df;
        TXSensorData.IMUSensor[4] = AV.y*R2Df;
        TXSensorData.IMUSensor[5] = AV.z*R2Df;
        // accx, accy, accz
        TXSensorData.IMUSensor[6] = LA.x;
        TXSensorData.IMUSensor[7] = LA.y;
        TXSensorData.IMUSensor[8] = LA.z;
    }
}

void DRCPlugin::OnUpdate(const common::UpdateInfo &){
    static int cnt = 0;

    for(int i=0; i<NO_OF_JOINTS; i++){
        refs[i] = 0;
    }

    if(connectionStatus){
        for(int i=0; i<NO_OF_JOINTS; i++){
            if(i == RHAND || i == LHAND || i == RWH || i == LWH)
                continue;
            refs[i] = RXJointData.JointReference[i]*D2Rf;
        }

        ref_RWH     += RXJointData.JointReference[RWH]*D2Rf/5.0;
        ref_LWH     += RXJointData.JointReference[LWH]*D2Rf/5.0;
        ref_RHAND   += RXJointData.JointReference[RHAND]*1.4708/40/2000;
        ref_LHAND   += RXJointData.JointReference[LHAND]*1.4708/40/2000;
        if(ref_RHAND>1.4708)    ref_RHAND = 1.4708;
        if(ref_RHAND<0)         ref_RHAND = 0;
        if(ref_LHAND>1.4708)    ref_LHAND = 1.4708;
        if(ref_LHAND<0)         ref_LHAND = 0;
    }

    // default offset
    refs[LEB] += -20*D2Rf;
    refs[REB] += -20*D2Rf;
    refs[RSR] += -15*D2Rf;
    refs[LSR] += 15*D2Rf;


    // move joints
    for(int i=0; i<NO_OF_JOINTS; i++){
        if(i == RHAND || i == LHAND || i == RWH || i == LWH)
            continue;
        JCon->SetPositionTarget(JPtrs[i]->GetScopedName(),refs[i]);
    }
    JCon->SetPositionTarget(JPtrs[RWH]->GetScopedName(), ref_RWH + home_ref_RWH);
    JCon->SetPositionTarget(JPtrs[LWH]->GetScopedName(), ref_LWH + home_ref_LWH);
    for(int i=0; i<9; i++){
        JCon->SetPositionTarget(JPtr_RHAND[i]->GetScopedName(), ref_RHAND);
        JCon->SetPositionTarget(JPtr_LHAND[i]->GetScopedName(), ref_LHAND);
    }
    JCon->SetPositionTarget(JPtr_RAFT->GetScopedName(),0);
    JCon->SetPositionTarget(JPtr_LAFT->GetScopedName(),0);
    JCon->SetPositionTarget(JPtr_RWFT->GetScopedName(),0);
    JCon->SetPositionTarget(JPtr_LWFT->GetScopedName(),0);

    //JPtr_Head->SetPosition(0, 0.0);
//    JCon->SetPositionTarget(JPtr_Head->GetScopedName(),0);
    JCon->Update();

    // adjust gain override
    adjustAllGain();

    // send sensor data==========================================
    for(int i=0; i<NO_OF_JOINTS; i++){
        if(i == RHAND || i == LHAND)
            continue;
        TXSensorData.JointCurrentPosition[i] = JPtrs[i]->GetAngle(0).Radian()*R2Df;

        if(i == RWH){
            TXSensorData.JointCurrentPosition[i] -= home_ref_RWH;
        }else if(i == LWH){
            TXSensorData.JointCurrentPosition[i] -= home_ref_LWH;
        }
    }
    TXSensorData.JointCurrentPosition[RHAND] = JPtr_RHAND[0]->GetAngle(0).Radian()*R2Df;
    TXSensorData.JointCurrentPosition[LHAND] = JPtr_LHAND[0]->GetAngle(0).Radian()*R2Df;

    // default offset
    TXSensorData.JointCurrentPosition[LEB] -= -20.0;
    TXSensorData.JointCurrentPosition[REB] -= -20.0;
    TXSensorData.JointCurrentPosition[RSR] -= -15.0;
    TXSensorData.JointCurrentPosition[LSR] -= 15.0;



    // RWFT, LWFT ------
    physics::JointWrench wrenchR = JPtr_RWFT->GetForceTorque(0);
    math::Vector3 RWF = wrenchR.body1Force;
    math::Vector3 RWT = wrenchR.body1Torque;
    filt_RWF = filt_alpha*RWF + (1-filt_alpha)*filt_RWF;
    filt_RWT = filt_alpha*RWT + (1-filt_alpha)*filt_RWT;
    if(connectionStatus){
        TXSensorData.FTSensor[2].force[0]   = filt_RWF.x;
        TXSensorData.FTSensor[2].force[1]   = filt_RWF.y;
        TXSensorData.FTSensor[2].force[2]   = filt_RWF.z;
        TXSensorData.FTSensor[2].torque[0]  = filt_RWT.x;
        TXSensorData.FTSensor[2].torque[1]  = filt_RWT.y;
        TXSensorData.FTSensor[2].torque[2]  = filt_RWT.z;
    }
    physics::JointWrench wrenchL = JPtr_LWFT->GetForceTorque(0);
    math::Vector3 LWF = wrenchL.body1Force;
    math::Vector3 LWT = wrenchL.body1Torque;
    filt_LWF = filt_alpha*LWF + (1-filt_alpha)*filt_LWF;
    filt_LWT = filt_alpha*LWT + (1-filt_alpha)*filt_LWT;
    if(connectionStatus){
        TXSensorData.FTSensor[3].force[0]   = filt_LWF.x;
        TXSensorData.FTSensor[3].force[1]   = filt_LWF.y;
        TXSensorData.FTSensor[3].force[2]   = filt_LWF.z;
        TXSensorData.FTSensor[3].torque[0]  = filt_LWT.x;
        TXSensorData.FTSensor[3].torque[1]  = filt_LWT.y;
        TXSensorData.FTSensor[3].torque[2]  = filt_LWT.z;
    }
    // -----------------
    //============================================================


    if(cnt%5==0){
        if(connectionStatus){
            common::Time simTime = model->GetWorld()->GetSimTime();
            ros::Time rosTime = ros::Time::now();

            TXSensorData.Sim_Time.sec = simTime.sec;
            TXSensorData.Sim_Time.nsec = simTime.nsec;
            TXSensorData.ROS_Time.sec = rosTime.sec;
            TXSensorData.ROS_Time.nsec = rosTime.nsec;
            write(sock, &TXSensorData, sizeof(TXSensorData));
        }

        // base_footprint tf----------------------
        math::Quaternion q = IMU->GetOrientation();
        math::Pose pos = model->GetWorldPose();
        broadcaster.sendTransform(
                    tf::StampedTransform(
                        tf::Transform(
                            tf::Quaternion(q.x, q.y, q.z, q.w),
                            tf::Vector3(pos.pos.x, pos.pos.y, pos.pos.z)),
                        ros::Time::now(),
                        "base_footprint",
                        "Body_TORSO"));
        //----------------------------------------


        //JPtrs[RWH]->SetAngle(0, 0.0);
        // Joint State Publish--------------------
//        joint_state.header.stamp = ros::Time::now();
//        for(int i=0; i<NO_OF_JOINTS; i++){
//            if(i == RHAND || i == LHAND){
//                joint_state.position[i] = 0.0;
//                continue;
//            }
//            joint_state.position[i] = JPtrs[i]->GetAngle(0).Radian();
//        }
//        for(int i=0; i<9; i++){
//            joint_state.position[NO_OF_JOINTS + i] = JPtr_LHAND[i]->GetAngle(0).Radian();
//            joint_state.position[NO_OF_JOINTS + 9 + i] = JPtr_RHAND[i]->GetAngle(0).Radian();
//        }
//        joint_state.position[NO_OF_JOINTS + 18 + 0] = JPtr_RAFT->GetAngle(0).Radian();
//        joint_state.position[NO_OF_JOINTS + 18 + 1] = JPtr_LAFT->GetAngle(0).Radian();
//        joint_state.position[NO_OF_JOINTS + 18 + 2] = JPtr_RWFT->GetAngle(0).Radian();
//        joint_state.position[NO_OF_JOINTS + 18 + 3] = JPtr_LWFT->GetAngle(0).Radian();
//        joint_state.position[NO_OF_JOINTS + 18 + 4] = 0.0;
//        joint_pub.publish(joint_state);
        //------------------------------------------
    }
    cnt++;
}

int DRCPlugin::CreateSocket(const char *addr, int port){
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if(sock == -1){
        return false;
    }
    client.sin_addr.s_addr = inet_addr(addr);
    client.sin_family = AF_INET;
    client.sin_port = htons(port);

    return true;
}


int DRCPlugin::Connect2Server(){
    if(connect(sock, (struct sockaddr*)&client, sizeof(client)) < 0){
        return false;
    }
    std::cout << "Client connect to server!! (DRC_PLUGIN)" << std::endl;
    return true;
}

void DRCPlugin::NewRXData(){
    // save previous refs
    for(int i=0; i<NO_OF_JOINTS; i++){
        prev_refs[i] = refs[i];
    }

    // refresh new refs
    memcpy(&(RXJointData), RXBuffer, RXDataSize);

    // calculate incremental refs
    for(int i=0; i<NO_OF_JOINTS; i++){
        inc_refs[i] = (RXJointData.JointReference[i]*D2Rf-prev_refs[i])/5.0;
    }

    new_ref_cnt = 0;
}


void *DRCPlugin::LANThread(void *_arg){
    DRCPlugin *dp = (DRCPlugin*)_arg;

    dp->threadWorking = true;

    unsigned int tcp_status = 0x00;
    int tcp_size = 0;
    int connectCnt = 0;

    int readDone = true;
    int dataType = 0;

    while(dp->threadWorking){
        usleep(100);
        if(tcp_status == 0x00){
            // If client was not connected
            if(dp->sock == 0){
                dp->CreateSocket(PODO_ADDR, PODO_PORT);
            }
            if(dp->Connect2Server()){
                tcp_status = 0x01;
                dp->connectionStatus = true;
                connectCnt = 0;
            }else{
                if(connectCnt%10 == 0)
                    std::cout << "Connect to Server Failed.." << std::endl;
                connectCnt++;
            }
            usleep(1000*1000);
        }
        if(tcp_status == 0x01){
            // If client was connected
            if(readDone == true){
                tcp_size = read(dp->sock, &dataType, sizeof(int));
                if(tcp_size == sizeof(int)){
                    readDone = false;
                }
            }
            else if(readDone == false){
                switch(dataType){
                case GAZEBO_TYPE_JOINT:
                    tcp_size = read(dp->sock, dp->RXBuffer, sizeof(DRC_GAZEBO_JOINT));
                    if(tcp_size == sizeof(DRC_GAZEBO_JOINT)){
                        dp->NewRXData();
                        readDone = true;
                        dataType = GAZEBO_TYPE_NO;
                    }
                    break;
                case GAZEBO_TYPE_GAINOVERRIDE:
                    tcp_size = read(dp->sock, &dp->GainOverrideCMD, sizeof(DRC_GAZEBO_GO_CMD));
                    if(tcp_size == sizeof(DRC_GAZEBO_GO_CMD)){
                        dp->setGainOverride(dp->GainOverrideCMD.joint, dp->GainOverrideCMD.gain, dp->GainOverrideCMD.timeMs);
                        readDone = true;
                        dataType = GAZEBO_TYPE_NO;
                    }
                    break;
                case GAZEBO_TYPE_HOME:
                    int joint;
                    tcp_size = read(dp->sock, &dp->HomeJoint, sizeof(int));
                    if(tcp_size == sizeof(int)){
                        dp->findHome(dp->HomeJoint);
                        readDone = true;
                        dataType = GAZEBO_TYPE_NO;
                    }
                    break;
                default:
                    std::cout << "DATA TYPE  :" << dataType << std::endl;
                    break;
                }
            }

            // disconnected..
            if(tcp_size == 0){
                tcp_status = 0x00;
                dp->connectionStatus = false;
                close(dp->sock);
                dp->sock = 0;
                std::cout << "Socket Disconnected.." << std::endl;
            }
        }
    }
    return NULL;
}



void DRCPlugin::setGainOverride(int joint, float gain, long msTime){
    GainOverride[joint].flag = false;
    GainOverride[joint].togo = gain;
    GainOverride[joint].init = GainOverride[joint].current;
    GainOverride[joint].delta = GainOverride[joint].togo - GainOverride[joint].current;
    GainOverride[joint].curCnt = 0;
    GainOverride[joint].goalCnt = msTime;
    GainOverride[joint].flag = true;
}

void DRCPlugin::moveGainOverride(int joint){
    if(GainOverride[joint].flag == true){
        GainOverride[joint].curCnt++;
        if(GainOverride[joint].goalCnt <= GainOverride[joint].curCnt){
            GainOverride[joint].goalCnt = GainOverride[joint].curCnt = 0;
            GainOverride[joint].current = GainOverride[joint].togo;
            GainOverride[joint].flag = false;
        }else{
            GainOverride[joint].current = GainOverride[joint].init + GainOverride[joint].delta*GainOverride[joint].curCnt/GainOverride[joint].goalCnt;
        }
    }
}

void DRCPlugin::adjustAllGain(){
    for(int i=0; i<NO_OF_JOINTS; i++){
        if(i == RHAND || i == LHAND)
            continue;
        if(GainOverride[i].flag == true){
            moveGainOverride(i);
            double gain = pow(0.9, GainOverride[i].current);
            JCon->SetPositionPID(JPtrs[i]->GetScopedName(),common::PID(gain*PIDGains[i][0],gain*PIDGains[i][1],gain*PIDGains[i][2]));
        }
    }
}

void DRCPlugin::findHome(int jNum){
    // Only for RWH & LWH
    if(jNum == RWH){
        home_ref_RWH += ref_RWH;
        ref_RWH = 0.0;
    }else if(jNum == LWH){
        home_ref_LWH += ref_LWH;
        ref_LWH = 0.0;
    }
}


GZ_REGISTER_MODEL_PLUGIN(DRCPlugin)
}
