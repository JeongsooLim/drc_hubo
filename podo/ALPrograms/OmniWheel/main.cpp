/*
 *	This program is generated from drc-podoal-template.
 *
 *	This AL-Process will activated by "PODO_Daemon" with appropriate "AL-Number".
 *	The AL-Number is determined at Core_config.db file at Share folder.
 *	And the AL-Process needs this AL-Number as an input parameter.
 *	So trying to open the AL-Process without any input parameter will terminate the process.
 *
 *	Please check your PODO_AL_NAME and Core_Config.db file.
 *	Actually the PODO_AL_NAME is used for recognizing the certain process and making it unique.
 *	So the same name of file with different path is allowed if the PODO_AL_NAMEs are different.
 *	But we recommend you that gather all of your build file in one folder (check the Core_Config.db file).
 *
 *	You can change the period of real-time thread by changing "RT_TIMER_PERIOD_MS" as another value in "rt_task_set_periodic".
 *	Please do not change "RT_TIMER_PERIOD_MS" value in "typedef.h".
 *	You can also change the priority of the thread by changing 4th parameter of "rt_task_create".
 *	In this function you need to care about the name of thread (the name should be unique).
 *
 *	Please do not change the "RBInitialize" function and fore code of main().
 *	You may express your idea in while loop of main & real-time thread.
 *
 *	Each AL-Process has its own command structure in Shared Memory.
 *	So, it can have its own command set.
 *	Make sure that the command set of each AL-process start from over than 100.
 *	Under the 100 is reserved for common command set.
 *
 *	Now, you can do everything what you want..!!
 *	If you have any question about PODO, feel free to contact us.
 *	Thank you.
 *
 *
 *
 *	Jungho Lee		: jungho77@rainbow.re.kr
 *	Jeongsoo Lim	: yjs0497@kaist.ac.kr
 *	Okkee Sim		: sim2040@kaist.ac.kr
 *
 *	Copy Right 2014 @ Rainbow Co., HuboLab of KAIST
 *
 */

// This AL is made by Hyoin Bae (HuboLab at KAIST)
// e-mail: pos97110@kaist.ac.kr

#include <QCoreApplication>

#include <iostream>
#include <string>
#include <sys/mman.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>


#include "joint.h"
#include "taskmotion.h"
#include "OmniWheelVariables.h"
#include "ManualCAN.h"

#define PODO_AL_NAME       "OMNIWHEEL_AL"

using namespace std;
doubles RWHList(120);
doubles LWHList(120);
inline void pushData(doubles &tar, double var){
    tar.push_back(var);
    tar.pop_front();
}

// Functions ===========================================
// Signal handler
void CatchSignals(int _signal);
int HasAnyOwnership();

// Real-time thread for control
void *RBTaskThread(void *);
void *RBFlagThread(void *);

// Initialization
int RBInitialize(void);
// =====================================================

// Variables ===========================================
// Shared memory
pRBCORE_SHM_COMMAND     sharedCMD;
pRBCORE_SHM_REFERENCE   sharedREF;
pRBCORE_SHM_SENSOR      sharedSEN;
pUSER_SHM       userData;

// RT task handler for control
ulong rtTaskCon;
ulong rtFlagCon;

// Program variable
int isTerminated;
int PODO_NO;
int PODO_NO_DAEMON = 0;
int PODO_NO_WALKREADY;
JointControlClass *joint;
// =====================================================


int CheckMotionOwned();
// WBIK functins-------------------------------------------------------------------------------- //
// Variables
int             WB_FLAG = false;
long            LimitedJoint;
long            LimitType;

// Functions
//int	PushCANMessage(MANUAL_CAN MCData);
//int RBJointGainOverride(unsigned int _canch, unsigned int _bno, unsigned int _override1, unsigned int _override2, unsigned int _duration);
//int RBBoardSetSwitchingMode(unsigned int _canch, unsigned int _bno, int _mode);
//int RBenableFrictionCompensation(unsigned int _canch, unsigned int _bno, unsigned int _enable1, unsigned int _enable2);



double odom_x = 0;
double odom_y = 0;
double odom_theta = 0;


TaskMotion      *WBmotion;



pthread_mutex_t m_suspendMutex;
pthread_cond_t  m_resumeCond;
bool            m_suspendFlag = 0;

void suspendThread(){
    pthread_mutex_lock(&m_suspendMutex);
    m_suspendFlag = 1;
    pthread_cond_wait(&m_resumeCond, &m_suspendMutex);
}
void resumeThread(){
    pthread_cond_signal(&m_resumeCond);
    m_suspendFlag = 0;
    pthread_mutex_unlock(&m_suspendMutex);
}
void checkSuspend(){
    pthread_mutex_lock(&m_suspendMutex);
    while(m_suspendFlag != 0)
        pthread_cond_wait(&m_resumeCond, &m_suspendMutex);
    pthread_mutex_unlock(&m_suspendMutex);
}


int     __IS_WORKING = false;
int     __IS_GAZEBO = false;

void CheckArguments(int argc, char *argv[]){
    int opt = 0;
    int podoNum = -1;
    while((opt = getopt(argc, argv, "g:p:")) != -1){
        switch(opt){
        case 'g':
            if(strcmp(optarg, "true")==0 || strcmp(optarg, "TRUE")==0){
                __IS_GAZEBO = true;
            }else if(strcmp(optarg, "false")==0 || strcmp(optarg, "FALSE")==0){
                __IS_GAZEBO = false;
            }else{
                FILE_LOG(logERROR) << optarg;
                FILE_LOG(logERROR) << "Invalid option for Gazebo";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
                FILE_LOG(logERROR) << "Use default value";
            }
            break;
        case 'p':
            podoNum = atoi(optarg);
            if(podoNum == 0){
                FILE_LOG(logERROR) << optarg;
                FILE_LOG(logERROR) << "Invalid option for AL";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
                FILE_LOG(logERROR) << "Use default value";
            }else{
                PODO_NO = podoNum;
            }
            break;
        case '?':
            if(optopt == 'g'){
                FILE_LOG(logERROR) << "Option for Gazebo";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
            }else if(optopt == 'p'){
                FILE_LOG(logERROR) << "Option for AL";
                FILE_LOG(logERROR) << "Valid options are \"Integer Values\"";
            }
        }
    }


    cout << endl;
    FILE_LOG(logERROR) << "===========AL Setting============";
    FILE_LOG(logWARNING) << argv[0];
    if(__IS_GAZEBO)     FILE_LOG(logWARNING) << "AL for Gazebo";
    else                FILE_LOG(logWARNING) << "AL for Robot";
    FILE_LOG(logWARNING) << "AL Number: " << PODO_NO;
    FILE_LOG(logERROR) << "=================================";
    cout << endl;
}


// --------------------------------------------------------------------------------------------- //
int main(int argc, char *argv[])
{
    // Termination signal ---------------------------------
    signal(SIGTERM, CatchSignals);   // "kill" from shell
    signal(SIGINT, CatchSignals);    // Ctrl-c
    signal(SIGHUP, CatchSignals);    // shell termination
    signal(SIGKILL, CatchSignals);

    // Block memory swapping ------------------------------
    mlockall(MCL_CURRENT|MCL_FUTURE);


    CheckArguments(argc, argv);
    if(PODO_NO == -1){
        FILE_LOG(logERROR) << "Please check the AL Number";
        FILE_LOG(logERROR) << "Terminate this AL..";
        return 0;
    }

//    // Get PODO No. ---------------------------------------
//    if(argc == 1){
//        FILE_LOG(logERROR) << "No input argument";
//        return 0;
//    }
//    else{
//        QString argStr;
//        argStr.sprintf("%s", argv[1]);
//        PODO_NO = argStr.toInt();
//        cout << "======================================================================" << endl;
//        cout << ">>> Process OmniWheel is activated..!!" << endl;
//        cout << ">>> PODO NAME: OMNIWHEEL_AL" << endl;
//        cout << ">>> PODO NO: " << PODO_NO << endl;
//        cout << "======================================================================" << endl;
//    }


    // Initialize RBCore -----------------------------------
    if( RBInitialize() == -1 )
        isTerminated = -1;

    // Getting AL number

    PODO_NO_WALKREADY = 3;
    FILE_LOG(logINFO) << "WalkReady AL number : " << PODO_NO_WALKREADY;
    usleep(500*1000);

    // WBIK Initialize--------------------------------------
    WBmotion = new TaskMotion(sharedREF, sharedSEN, sharedCMD, joint);

    userData->odom_data[0] = 0;
    userData->odom_data[1] = 0;
    userData->odom_data[2] = 0;
    userData->odom_data[3] = 0;
    userData->odom_data[4] = 0;
    userData->odom_data[5] = 0;


    // User command cheking --------------------------------
    while(isTerminated == 0){
        usleep(100*1000);
//        FILE_LOG(logINFO) << odom_x << ", " << odom_y;
        switch(sharedCMD->COMMAND[PODO_NO].USER_COMMAND)
        {
            //---------------------------------------------------------------------------------
            // Wheel movement
            //---------------------------------------------------------------------------------
        case OMNIWHEEL_AL_GOTODES:
            REAL_MODE = true;
            // Char
            // 3 : velocity change
            // 4 : nowait Flag
            // 5 : tank mode Flag
            // 6 : direct compensation Flag

            //sharedData->STATE_COMMAND = TCMD_WHEEL_MOVE_START;

            STATUS_FLAG=STATUS_OPERATE;

            _des_x=sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            _des_y=sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
            _des_a=sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];

            // NAN CHECK
            if(isnan(_des_x) || isnan(_des_y) || isnan(_des_a)){
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = OMNIWHEEL_AL_NO_ACT;
                break;
                printf("----------!!!---- Is NAN OUT----!!!----------\n");
            }

            velChangeFlag = false;
            noWaitFlag = false;
            directCompenFlag = false;

            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[3] == 3){
                velChangeFlag = true;
                sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[3] =0;
            }else{
                velChangeFlag = false;
            }
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[4] == 4){
                noWaitFlag = true;
                sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[4] =0;
            }else{
                noWaitFlag = false;
            }

            if(velChangeFlag == true){
                velChangeFlag = false;
                _max_wheel_rot = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[3];
                if(isnan(_max_wheel_rot))
                    _max_wheel_rot = SET_MAX_WHEEL;
                if(_max_wheel_rot >(SET_MAX_WHEEL+1) || _max_wheel_rot < 100)
                    _max_wheel_rot = SET_MAX_WHEEL;
            }
            else
                _max_wheel_rot = SET_MAX_WHEEL;

            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[5] == 5){
                joint->RefreshToCurrentReference();
                joint->SetAllMotionOwnerWHEELandWST();
                usleep(5*1000);
                sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[5]=0;
                Tank_Mode_Flag = true;
                Tank_delta =joint->GetJointRefAngle(WST);
            }else{
                joint->RefreshToCurrentReference();
                joint->SetAllMotionOwnerWHEEL();
            }

            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[6] == 6){
                directCompenFlag = true;
                sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[6] =0;
            }else{
                directCompenFlag = false;
            }
            directCompenFlag = true;


            OmniPathGenerator(_des_x, _des_y, _des_a, 1);

            maxLinearVel=_max_wheel_rot*D2R*_wheel_radi;//m/s - 0.30823277 m/s when 300
            maxAngularVel=((maxLinearVel*0.7)/(_robot_width/2.))*R2D;//deg/s - 65.5555 deg/s when 300
            if(joint->GetJointRefAngle(WST) > 90 || joint->GetJointRefAngle(WST)<-90){
                maxLinearVel = maxLinearVel*0.7;
                maxAngularVel = maxAngularVel*0.7;
                maxLinearAcc=0.13;//maxLinearVel/1.2;
                maxAngularAcc=25;//maxAngularVel/1.5;
            }else{
                maxLinearAcc=0.16;//maxLinearVel/1.2;
                maxAngularAcc=30;//maxAngularVel/1.5;
            }

            _OW_YAW_VALUE =0.;
            _OW_YAW_DES = 0.;

            if(_GAIN_HIP_FLAG == true){
                MCJointGainOverride(0,2,1,_GAIN_HIP_PITCH,200);
                MCJointGainOverride(1,8,1,_GAIN_HIP_PITCH,200);
                usleep(210*1000);
            }

            if(_EM_Stop_Flag==0)
            {
                if(_localCommand==IDLE_STATUS)
                {
                    _localCount = 0;
                    _isFirst = 1;
                    _localMotionNum = MOTION_1;
                    _localCommand = WHEEL_OPERATION;
                }
                else
                    printf(">>> Error - Other Wheel command is activated!\n");
            }
            else
                printf(">>> Error - EM Stop is activated!!\n");

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = OMNIWHEEL_AL_NO_ACT;
            break;
        case OMNIWHEEL_AL_RADIUS:
            STATUS_FLAG=STATUS_OPERATE;

            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwnerWHEEL();
            printf("OW - Radius move mode...!!!\n");
            {
            WMR_dir=sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0];//1 is right, 0 is left
            WMR_fb=sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1];//1 is go foward -1 is go backward
            WMR_radius=sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            WMR_angle=sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
            //printf("OW - Radius mode: %d side %d %lf m %lf deg\n", WMR_dir, WMR_fb, WMR_radius, WMR_angle);

            double WM_radius_vel = 150.;
            radius_velChangeFlag =false;
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[3] ==3){
                radius_velChangeFlag = true;
                sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[3] = 0;
            }else{
                radius_velChangeFlag = false;
            }
            if(radius_velChangeFlag == true){
                radius_velChangeFlag = false;
                WM_radius_vel = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[3];
            }else
                WM_radius_vel = 150.;


            WMR_omega_global_max=(WM_radius_vel*D2R*_wheel_radi)/WMR_radius*R2D;
            WMR_alpha_global_max=WMR_omega_global_max/1.5;

            if(_EM_Stop_Flag==0)
            {
                if(_localCommand==IDLE_STATUS)
                {
                    _localCount = 0;
                    _isFirst = 1;
                    _localMotionNum = RADIUS_1;
                    _localCommand = WHEEL_RADIUSMODE;
                }
                else
                    printf(">>> Error - Other Wheel command is activated!\n");
            }
            else
                printf(">>> Error - EM Stop is activated!!\n");
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = OMNIWHEEL_AL_NO_ACT;
            break;
        case OMNIWHEEL_AL_VELMODE:
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwnerWHEEL();
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]==0)
            {
                // Stop
                _OW_RIGHT=0.;
                _OW_LEFT=0.;
                _OW_YAW_DES = 0.;
                _OW_YAW_VALUE =0.;
                _OW_DIRCT =0.;
                STATUS_FLAG=STATUS_IDLE;
                printf("OW - Move Stop...!!!\n");
                //sharedData->STATE_COMMAND = TCMD_WHEEL_MOVE_DONE;
                _localCommand=IDLE_STATUS;
                _localMotionNum=MOTION_NONE;
                _localCount=0;
                joint->SetMoveJoint(RWH, 0, 100, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LWH, 0, 100, MOVE_ABSOLUTE);
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]==1)
            {
                // Direct
                STATUS_FLAG=STATUS_OPERATE;
                printf("OW - Move Direct...!!!\n");
                double OW_des_vel=sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                double OW_des_vel_step=OW_des_vel/200.;
                _localCommand=WHEEL_VELMODE;
                _localMotionNum=MOTION_NONE;
                _localCount=0;
                joint->SetMoveJoint(RWH, OW_des_vel_step, 1000, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LWH, OW_des_vel_step, 1000, MOVE_ABSOLUTE);
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]==2)
            {
                // Rotate
                STATUS_FLAG=STATUS_OPERATE;
                printf("OW - Move Rotate...!!!\n");
                double OW_des_vel=sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                double OW_des_vel_step=OW_des_vel/200.;
                _localCommand=WHEEL_VELMODE;
                _localMotionNum=MOTION_NONE;
                _localCount=0;
                joint->SetMoveJoint(RWH, OW_des_vel_step, 1000, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LWH, -OW_des_vel_step, 1000, MOVE_ABSOLUTE);
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]==3)
            {
                SaveFile();
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]==4)
            {
                // Init Wheel Home
                InitWheelHome();
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]==5)
            {
                saveFlag = 1;
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = OMNIWHEEL_AL_NO_ACT;
            break;
        case OMNIWHEEL_AL_MANUAL:
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]==1)//manual move on
            {
                joint->RefreshToCurrentReference();
                joint->SetAllMotionOwnerWHEEL();

                saveFlag = 1;//for BLE

                printf("OW - Manual mode Start...!!!\n");
                _localCommand=WHEEL_MANUALMODE;
                _localMotionNum=MOTION_NONE;
                _localCount=0;

                STATUS_FLAG=STATUS_OPERATE;

            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]==0)//manual move off
            {
                printf("OW - Manual mode Stop...!!!\n");
                L_JOG_RL=0;
                AROW_RL=0;
                R_JOG_UD=0;
                sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0]=0;
                sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1]=0;
                sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[2]=0;

                _localCommand=IDLE_STATUS;
                _localMotionNum=MOTION_NONE;
                _localCount=0;

                STATUS_FLAG=STATUS_IDLE;
                joint->SetMoveJoint(RWH, 0, 1200, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LWH, 0, 1200, MOVE_ABSOLUTE);
                usleep(1200*1000);

                SaveFile();
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = OMNIWHEEL_AL_NO_ACT;
            break;
            //---------------------------------------------------------------------------------
            // Posture change
            //---------------------------------------------------------------------------------
        case OMNIWHEEL_AL_CONTROL:
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]==0)
            {
                // Gain Over
                MCsetFrictionParameter(0,4,2,1000,_GAIN_RWH,0);
                MCsetFrictionParameter(1,10,2,1000,_GAIN_LWH,0);
                MCBoardSetSwitchingMode(0,4,1);
                MCBoardSetSwitchingMode(1,10,1);
                MCenableFrictionCompensation(0,4,2,ENABLE);
                MCenableFrictionCompensation(1,10,2,ENABLE);
                MCJointGainOverride(0,4,2,100,1);
                MCJointGainOverride(1,10,2,100,1);
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]==1)
            {
                // Gain return
                MCenableFrictionCompensation(0,4,2,DISABLE);
                MCenableFrictionCompensation(1,10,2,DISABLE);
                MCBoardSetSwitchingMode(0,4,0);
                MCBoardSetSwitchingMode(1,10,0);
                MCJointGainOverride(0,4,2,0,3000);
                MCJointGainOverride(1,10,2,0,3000);
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]==2)
            {
                printf("Knee Gain Over Start...!!!\n");
                // Knee Gain over start
                Knee_Gain_Over();
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]==3)
            {
                printf("Knee Gain Over Stop...!!!\n");
                // Knee Gain over return
                Knee_Gain_Return();
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]==4)
            {
                joint->RefreshToCurrentReference();
                joint->SetAllMotionOwnerWHEEL();

                // IMU null
                _OW_YAW_VALUE = 0.;
                _OW_YAW_DES = 0.;
                printf("OW - Yaw nulling\n");
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]==5)
            {
                joint->RefreshToCurrentReference();
                joint->SetAllMotionOwnerWHEEL();

                // IMU null
                yawFlag = 1;
                printf("OW - Yaw Start\n");
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]==10)
            {
                char onoff = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1];
                if(onoff ==1){
                    REAL_MODE = true;
                    printf("Mode is changed to Real...!!!\n");
                }else if(onoff ==0){
                    REAL_MODE = false;
                    printf("Mode is changed to Test...!!!\n");
                }
                //sharedData->STATE_COMMAND = TCMD_WHEEL_REAL_TEST_MODE_SET;
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 20){
                // Hip Pitch
                char onoff = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1];
                if(onoff == 1){//on
                    printf("Pitch Manual Gain ON\n");
                    MCJointGainOverride(0,2,1,_GAIN_HIP_PITCH,200);
                    MCJointGainOverride(1,8,1,_GAIN_HIP_PITCH,200);
                }else if(onoff ==0){
                    printf("Pitch Manual Gain OFF\n");
                    MCJointGainOverride(0,2,1,0,1000);
                    MCJointGainOverride(1,8,1,0,1000);
                }
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 21){
                char onoff = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1];
                if(onoff == 1){//on
                    printf("Pitch Suspen mode ON !!!\n");
                    _GAIN_HIP_FLAG = true;
                }else if(onoff ==0){
                    printf("Pitch Suspen mode OFF !!!\n");
                    _GAIN_HIP_FLAG = false;
                }
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = OMNIWHEEL_AL_NO_ACT;
            break;
        case OMNIWHEEL_AL_CHANGEPOS:
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();

            //saveFlag = 1;

            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]==0)
            {
                double check_1 = joint->GetJointRefAngle(RKN);
                double check_2 = joint->GetJointRefAngle(LKN);
                if((check_1 >130) || (check_2 >130)){
                    printf("Wrong Wheel-Walk pos change Command...!!!!!!!\n");
                    sharedCMD->COMMAND[PODO_NO].USER_COMMAND = OMNIWHEEL_AL_NO_ACT;
                    usleep(5*1000);
                    break;
                }
                //sharedData->STATE_COMMAND = TCMD_WHEEL_POS_CHANGE_WALK2WH_START;
                STATUS_FLAG=STATUS_OPERATE;
                printf("-------------GOTO wheel pos transform---------!!!!\n");

                WB_FLAG = false;
                usleep(10*1000);
                joint->SetMoveJoint(RHY, 0., 450, MOVE_ABSOLUTE);
                joint->SetMoveJoint(RHR, 0., 450, MOVE_ABSOLUTE);
                joint->SetMoveJoint(RAR, 0., 450, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LHY, 0., 450, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LHR, 0., 450, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LAR, 0., 450, MOVE_ABSOLUTE);
                usleep(460*1000);

                pos_change_count=0;
                WHEELtoWALK_FLAG=false;
                WALKtoWHEEL_FLAG=true;

                usleep(20*1000);
                int whilecnt = 0;
                while(1){
                    if(WALKtoWHEEL_FLAG == false){
                        printf("Pos change success...!!!\n");
                        break;
                    }
                    whilecnt++;
                    if(whilecnt>=600){
                        printf("Pos change time out...!!!\n");
                        break;
                    }
                    usleep(50*1000);
                }

//                if(REAL_MODE == true)
//                    Knee_Gain_Over();
                //sharedData->STATE_COMMAND = TCMD_WHEEL_POS_CHANGE_WALK2WH_DONE;
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]==1)
            {
                double check_1 = joint->GetJointRefAngle(RKN);
                double check_2 = joint->GetJointRefAngle(LKN);
                if((check_1 <130) || (check_2 <130)){
                    printf("Wrong Wheel-Walk pos change Command...!!!!!!!\n");
                    sharedCMD->COMMAND[PODO_NO].USER_COMMAND = OMNIWHEEL_AL_NO_ACT;
                    usleep(5*1000);
                    break;
                }

                //sharedData->STATE_COMMAND = TCMD_WHEEL_POS_CHANGE_WH2WALK_START;

                double check_3 = fabs(joint->GetJointRefAngle(RSP)-40);
                double check_4 = fabs(joint->GetJointRefAngle(REB)+130);
                if((check_3>10.) || (check_4>10)){
                    WB_FLAG = false;
                    usleep(10*1000);

//                    double temp_ms = 2000;
//                    joint->SetMoveJoint(RSP, 40.0, temp_ms, MOVE_ABSOLUTE);
//                    joint->SetMoveJoint(RSR, 10.0, temp_ms, MOVE_ABSOLUTE);
//                    joint->SetMoveJoint(RSY, 0.0, temp_ms, MOVE_ABSOLUTE);
//                    joint->SetMoveJoint(REB, -130, temp_ms, MOVE_ABSOLUTE);
//                    joint->SetMoveJoint(RWY, 0.0, temp_ms, MOVE_ABSOLUTE);
//                    joint->SetMoveJoint(RWP, 20.0, temp_ms, MOVE_ABSOLUTE);

//                    joint->SetMoveJoint(LSP, 40.0, temp_ms, MOVE_ABSOLUTE);
//                    joint->SetMoveJoint(LSR, -10.0, temp_ms, MOVE_ABSOLUTE);
//                    joint->SetMoveJoint(LSY, 0.0, temp_ms, MOVE_ABSOLUTE);
//                    joint->SetMoveJoint(LEB, -130.0, temp_ms, MOVE_ABSOLUTE);
//                    joint->SetMoveJoint(LWY, 0.0, temp_ms, MOVE_ABSOLUTE);
//                    joint->SetMoveJoint(LWP, 20.0, temp_ms, MOVE_ABSOLUTE);

//                    //joint->SetMoveJoint(RWY2, 0.0, temp_ms, MOVE_ABSOLUTE);
//                    //joint->SetMoveJoint(LWY2, 0.0, temp_ms, MOVE_ABSOLUTE);

//                    usleep((temp_ms+10)*1000);
                }


                STATUS_FLAG=STATUS_OPERATE;
                printf("-------------GOTO walkready pos transform---------!!!!\n");

                if(REAL_MODE == true)
                    Knee_Gain_Return();

                pos_change_count=0;
                WALKtoWHEEL_FLAG=false;
                WHEELtoWALK_FLAG=true;

                usleep(20*1000);
                int whilecnt = 0;
                while(1){
                    if(WHEELtoWALK_FLAG == false){
                        printf("Pos change success...!!!\n");
                        break;
                    }
                    whilecnt++;
                    if(whilecnt>=600){
                        printf("Pos change time out...!!!\n");
                        break;
                    }
                    usleep(50*1000);
                }
                //sharedData->STATE_COMMAND = TCMD_WHEEL_POS_CHANGE_WH2WALK_DONE;
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] ==2)
            {
                WB_FLAG = false;
                usleep(10*1000);
                double postime = 3000.;
                joint->SetMoveJoint(RHY, 0., postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(RHR, 0., postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(RHP, _POS_HIP_PITCH, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(RKN, 145, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(RAP, _POS_ANKLE_PITCH, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(RAR, 0., postime, MOVE_ABSOLUTE);

                joint->SetMoveJoint(LHY, 0., postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LHR, 0., postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LHP, _POS_HIP_PITCH, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LKN, 145, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LAP, _POS_ANKLE_PITCH, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LAR, 0., postime, MOVE_ABSOLUTE);
                usleep(3000*1000);
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] ==3)
            {
                WB_FLAG = false;
                usleep(10*1000);
                double postime = 3000.;
                joint->SetMoveJoint(RHY, 0., postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(RHR, 0., postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(RHP, -46.45, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(RKN, 145, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(RAP, _POS_ANKLE_PITCH, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(RAR, 0., postime, MOVE_ABSOLUTE);

                joint->SetMoveJoint(LHY, 0., postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LHR, 0., postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LHP, -46.45, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LKN, 145, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LAP, _POS_ANKLE_PITCH, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LAR, 0., postime, MOVE_ABSOLUTE);
                usleep(3000*1000);
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] ==4)
            {
                char mode = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1];
                WB_FLAG = false;
                usleep(10*1000);

                if(mode == 1){
                    //up

                    // gain return
                    Knee_Gain_Return();

                    // move joint
                    double postime = 2500;
                    double des_HP = -46.45 + delta_pos_angle;
                    double des_KN = 145 - delta_pos_angle;

                    joint->SetMoveJoint(RHY, 0., postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RHR, 0., postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RHP, des_HP, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RKN, des_KN, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RAP, _POS_ANKLE_PITCH, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RAR, 0., postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LHY, 0., postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LHR, 0., postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LHP, des_HP, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LKN, des_KN, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LAP, _POS_ANKLE_PITCH, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LAR, 0., postime, MOVE_ABSOLUTE);
                    usleep(postime*1000);
                }else if(mode ==-1){
                    //down

                    // move joint
                    double postime = 2500;
                    double des_HP = _POS_HIP_PITCH;
                    double des_KN = 145;
                    joint->SetMoveJoint(RHY, 0., postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RHR, 0., postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RHP, des_HP, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RKN, des_KN, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RAP, _POS_ANKLE_PITCH, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RAR, 0., postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LHY, 0., postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LHR, 0., postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LHP, des_HP, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LKN, des_KN, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LAP, _POS_ANKLE_PITCH, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LAR, 0., postime, MOVE_ABSOLUTE);
                    usleep(postime*1000);

                    // gain release
                    Knee_Gain_Over();
                }
                //userData->WheelUpPosDoneFlag = true;
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] ==5)
            {
                char mode = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1];
                float del_ang = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[0];
                WB_FLAG = false;
                usleep(10*1000);
                if(mode == 1){//up
                    // gain return
                    Knee_Gain_Return();

                    // move joint
                    double postime = 2500;
                    double des_HP = -46.45 + del_ang;
                    double des_KN = 145 - del_ang;

                    joint->SetMoveJoint(RHY, 0., postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RHR, 0., postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RHP, des_HP, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RKN, des_KN, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RAP, _POS_ANKLE_PITCH, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RAR, 0., postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LHY, 0., postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LHR, 0., postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LHP, des_HP, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LKN, des_KN, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LAP, _POS_ANKLE_PITCH, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LAR, 0., postime, MOVE_ABSOLUTE);
                    usleep(postime*1000);

                }else if(mode ==0){//down
                    // move joint
                    double postime = 2500;
                    double des_HP = _POS_HIP_PITCH;
                    double des_KN = 145;
                    joint->SetMoveJoint(RHY, 0., postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RHR, 0., postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RHP, des_HP, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RKN, des_KN, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RAP, _POS_ANKLE_PITCH, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RAR, 0., postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LHY, 0., postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LHR, 0., postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LHP, des_HP, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LKN, des_KN, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LAP, _POS_ANKLE_PITCH, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LAR, 0., postime, MOVE_ABSOLUTE);
                    usleep(postime*1000);

                    // gain release
                    Knee_Gain_Over();
                }
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = OMNIWHEEL_AL_NO_ACT;
            break;
        case OMNIWHEEL_AL_ROS:

            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwnerWHEEL();

           // saveFlag = 1;//for BLE

            printf("OMNIWHEEL ROS ...!!!\n");
            printf("OMNIWHEEL ROS ...!!!\n");
            printf("OMNIWHEEL ROS ...!!!\n");

            _localCommand=WHEEL_ROSMODE;
            _localMotionNum=MOTION_NONE;
            _localCount=0;

            STATUS_FLAG=STATUS_OPERATE;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = OMNIWHEEL_AL_NO_ACT;
            break;

        default:
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = OMNIWHEEL_AL_NO_ACT;
            break;
        }
    }
    cout << ">>> Process OmniWheel is terminated..!!" << endl;
    return 0;
}
// --------------------------------------------------------------------------------------------- //


float vel_x_filt = 0.0;
float vel_w_filt = 0.0;
// --------------------------------------------------------------------------------------------- //
void *RBTaskThread(void *)
{
     while(isTerminated == 0)
    {
        checkSuspend();

         //=======================================
         // Calculate Odom for SLAM & Navigation
         //=======================================
         //const float D2Rf = 0.0174533f;
         double vr = sharedSEN->ENCODER[MC_GetID(RWH)][MC_GetCH(RWH)].CurrentVelocity*D2Rf;
         double vl = sharedSEN->ENCODER[MC_GetID(LWH)][MC_GetCH(LWH)].CurrentVelocity*D2Rf;
//         FILE_LOG(logWARNING) << vr << ", " << vl;
//         double w_sp = 0.40;
//         double w_r = 0.051;
         double w_sp = 0.37;
         double w_r = 0.06;
         double dt = RT_TIMER_PERIOD_MS/1000.0;

         double sl = vl*w_r*dt;
         double sr = vr*w_r*dt;
         double theta = (sr-sl)/w_sp;

         double dx = (sl+sr)/2.0*cos(sharedSEN->FOG.Yaw*D2Rf + sharedSEN->FOG.YawVel*dt/2.0);
         double dy = (sl+sr)/2.0*sin(sharedSEN->FOG.Yaw*D2Rf + sharedSEN->FOG.YawVel*dt/2.0);
         double dtheta = sharedSEN->FOG.YawVel*dt;

         odom_x += dx;
         odom_y += dy;
         odom_theta = sharedSEN->FOG.Yaw*D2Rf;

         double w = sharedSEN->FOG.YawVel;
         double v = sqrt(dx*dx+dy*dy)/dt;
         double vx = dx/dt;
         double vy = dy/dt;
         double vth = w;

         userData->odom_data[0] = odom_x;
         userData->odom_data[1] = odom_y;
         userData->odom_data[2] = odom_theta;
         userData->odom_data[3] = vx;
         userData->odom_data[4] = vy;
         userData->odom_data[5] = vth;
         //=======================================


        switch(_localCommand)
        {
        case WHEEL_OPERATION:
            inicator = OW_Operation_in_RealTime(_localCount, _localMotionNum);
            if(inicator==0)
                ;
            else if(inicator==1)
            {
                if(_localMotionNum == MOTION_1)
                {
                    _localCount = 0;

                    double des_angle = _path_plan[0][0] * _path_plan[0][1];
                    double cur_angle = _OW_YAW_DES;
                    double delta = des_angle - cur_angle;
                    if(delta > 90.)
                        delta = 90.;
                    else if(delta < -90.)
                        delta = -90.;
                    if(fabs(delta)<0.1){
                        _compen_1[0] = 0.;
                        _compen_1[1] = 1.;
                    }else{
                        _compen_1[0] = fabs(delta);
                        _compen_1[1] = sign(delta);
                    }
                    _localMotionNum = COMPEN_1;
                    //printf("Compensation mode 1 : %lf %lf\n", _compen_1[0], _compen_1[1]);
                }
                else if(_localMotionNum == COMPEN_1)
                {
                    _localCount = 0;

                    double des_angle = _compen_1[0] * _compen_1[1];
                    double cur_angle = _OW_YAW_DES;
                    double delta = des_angle - cur_angle;
                    if(delta > 90.)
                        delta = 90.;
                    else if(delta < -90.)
                        delta = -90.;
                    if(fabs(delta)<0.1){
                        _compen_1_1[0] = 0.;
                        _compen_1_1[1] = 1.;
                    }else{
                        _compen_1_1[0] = fabs(delta);
                        _compen_1_1[1] = sign(delta);
                    }
                    _localMotionNum = COMPEN_1_1;
                    //printf("Compensation mode 1-1 : %lf %lf\n", _compen_1_1[0], _compen_1_1[1]);
                }
                else if(_localMotionNum == COMPEN_1_1)
                {
                    printf("POSITION 1/3 END...!!!\n");
                    _localCount = 0;
                    _localMotionNum = MOTION_2;
                }
                else if(_localMotionNum == MOTION_2)
                {
                    printf("POSITION 2/3 END...!!!\n");
                    if(directCompenFlag == true)
                    {
                        double cur_error = -_OW_DIRCT;
                        double ori_next = _path_plan[2][0] * _path_plan[2][1];
                        double new_next = ori_next + cur_error;
                        _path_plan[2][0] = fabs(new_next);
                        _path_plan[2][1] = sign(new_next);
                        directCompenFlag = false;
                    }

                    _localCount = 0;
                    _localMotionNum = MOTION_3;
                }
                else if(_localMotionNum == MOTION_3)
                {
                    _localCount = 0;

                    double des_angle = _path_plan[2][0] * _path_plan[2][1];
                    double cur_angle = _OW_YAW_DES;
                    double delta = des_angle - cur_angle;
                    if(delta > 90.)
                        delta = 90.;
                    else if(delta < -90.)
                        delta = -90.;
                    if(fabs(delta)<0.1){
                        _compen_3[0] = 0.;
                        _compen_3[1] = 1.;
                    }else{
                        _compen_3[0] = fabs(delta);
                        _compen_3[1] = sign(delta);
                    }
                    _localMotionNum = COMPEN_3;
                }
                else if(_localMotionNum == COMPEN_3)
                {
                    _localCount = 0;

                    double des_angle = _compen_3[0] * _compen_3[1];
                    double cur_angle = _OW_YAW_DES;
                    double delta = des_angle - cur_angle;
                    if(delta > 90.)
                        delta = 90.;
                    else if(delta < -90.)
                        delta = -90.;
                    if(fabs(delta)<0.1){
                        _compen_3_1[0] = 0.;
                        _compen_3_1[1] = 1.;
                    }else{
                        _compen_3_1[0] = fabs(delta);
                        _compen_3_1[1] = sign(delta);
                    }
                    _localMotionNum = COMPEN_3_1;
                }
                else if(_localMotionNum == COMPEN_3_1)
                {
                    printf("POSITION 3/3 END...!!!\n");

                    Tank_Mode_Flag = false;
                    _localCount = 0;
                    _localCommand = IDLE_STATUS;
                    _localMotionNum = MOTION_NONE;
                    STATUS_FLAG=STATUS_IDLE;

                    //sharedData->STATE_COMMAND = TCMD_WHEEL_MOVE_DONE;

                    //userData->WheelDoneFlag = true;
                }
                else
                    _localCommand = IDLE_STATUS;

            }
            else
                _localCommand = IDLE_STATUS;

            if(Tank_Mode_Flag == true)
                joint->SetMoveJoint(WST, Tank_delta, 5, MOVE_ABSOLUTE);

            // Actual Movement
            OW_WHEEL_MOVEMENT();

            break;
        case WHEEL_RADIUSMODE:
            inicator_radius = OW_Operation_in_RealTime_Radius(_localCount, _localMotionNum);
            if(inicator_radius==0)
                ;
            else if(inicator_radius==1)
            {
                if( _localMotionNum == RADIUS_1)
                {
                    printf("RADIUS 1/3 END...!!!\n");
                    _localCount = 0;

                    double des_angle;
                    if(WMR_dir ==1)//right
                        des_angle = WMR_angle*WMR_fb*-1.;
                    else//left
                        des_angle = WMR_angle*WMR_fb*1.;

                    double cur_angle = _OW_YAW_DES;
                    double delta = cur_angle - des_angle;
                    if(delta > 90.)
                        delta = 90.;
                    else if(delta < -90.)
                        delta = -90.;
                    if(fabs(delta)<0.3){
                        _r_compen_1[0] = 0.;
                        _r_compen_1[1] = 1.;
                    }else{
                        _r_compen_1[0] = fabs(delta);
                        if(WMR_dir==1)//right
                            _r_compen_1[1] = sign(delta);
                        else
                            _r_compen_1[1] = -sign(delta);
                    }
                    _localMotionNum = RADIUS_2;
                }
                else if(_localMotionNum == RADIUS_2)
                {
                    printf("RADIUS 2/3 END...!!!\n");;
                    _localCount = 0;

                    double des_angle;
                    if(WMR_dir ==1)//right
                        des_angle = _r_compen_1[0]*_r_compen_1[1]*-1.;
                    else//left
                        des_angle = _r_compen_1[0]*_r_compen_1[1]*1.;

                    double cur_angle = _OW_YAW_DES;
                    double delta = cur_angle - des_angle;
                    if(delta > 90.)
                        delta = 90.;
                    else if(delta < -90.)
                        delta = -90.;
                    if(fabs(delta)<0.3){
                        _r_compen_2[0] = 0.;
                        _r_compen_2[1] = 1.;
                    }else{
                        _r_compen_2[0] = fabs(delta);
                        if(WMR_dir==1)//right
                            _r_compen_2[1] = sign(delta);
                        else
                            _r_compen_2[1] = -sign(delta);
                    }
                    _localMotionNum = RADIUS_3;
                }
                else if(_localMotionNum == RADIUS_3)
                {
                    printf("RADIUS 3/3 END...!!!\n");;
                    _localCount = 0;
                    _localCommand = IDLE_STATUS;
                    _localMotionNum = MOTION_NONE;
                    STATUS_FLAG =STATUS_IDLE;
                    //userData->WheelDoneFlag = true;
                }
                else
                    _localCommand = IDLE_STATUS;
            }
            else
                _localCommand = IDLE_STATUS;

            // Actual Movement
            OW_WHEEL_MOVEMENT();

            break;
        case WHEEL_VELMODE:
            ;// Vel mode is operating  now...
            break;
        case WHEEL_ROSMODE:
        {
            float vr, vl;

//            vel_x_filt = vel_x_filt * 0.3 + userData->vel_cmd[0] * 0.7;
//            vel_w_filt = vel_w_filt * 0.3 + userData->vel_cmd[1] * 0.7;

            vel_x_filt = userData->vel_cmd[0];
            vel_w_filt = userData->vel_cmd[1];


            vr = 0.5*(2.0*vel_x_filt - w_sp*vel_w_filt)/w_r;
            vl = 0.5*(2.0*vel_x_filt + w_sp*vel_w_filt)/w_r;


//            w_sp = 0.37;
//            w_r = 0.06;

//            vr = 0.5*(2.0*sharedData->vel_cmd[0] - w_sp*sharedData->vel_cmd[1])/w_r;
//            vl = 0.5*(2.0*sharedData->vel_cmd[0] + w_sp*sharedData->vel_cmd[1])/w_r;

            vr *= R2Df;
            vl *= R2Df;
            vr /= 200.0;
            vl /= 200.0;

//            FILE_LOG(logINFO) << sharedData->vel_cmd[0] << ", " << sharedData->vel_cmd[1];

            joint->SetMoveJoint(RWH, vr, 5, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LWH, vl, 5, MOVE_ABSOLUTE);
        }
            break;
        case WHEEL_MANUALMODE:
            // Wheel JoyStick Manual Move
            L_JOG_RL=sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
            AROW_RL=sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1];
            R_JOG_UD=sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[2];
            printf("Received : %d %d %d\n", L_JOG_RL, AROW_RL, R_JOG_UD);

            if((L_JOG_RL==L_JOG_RL_pre) && (AROW_RL==AROW_RL_pre) && (R_JOG_UD==R_JOG_UD_pre)){
                JOG_SAME_COUNT++;
            }else{
                JOG_SAME_COUNT=0;
            }

            L_JOG_RL_pre = L_JOG_RL;
            AROW_RL_pre = AROW_RL;
            R_JOG_UD_pre = R_JOG_UD;

            if(JOG_SAME_COUNT >= 12000){
                L_JOG_RL = 0;
                AROW_RL =0;
                R_JOG_UD =0;
            }

            WheelMoveManual();
            CalculateMovingEverage();

            joint->SetMoveJoint(RWH, RWHnow, 5, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LWH, LWHnow, 5, MOVE_ABSOLUTE);
            break;
        }

        _localCount++;
        if(_localCount>=1000000)
            _localCount=0;

        // Pos Change Function
        if(WALKtoWHEEL_FLAG==true)
        {
            WALKtoWHEEL_FUNCTION_FAST();
        }
        if(WHEELtoWALK_FLAG==true)
        {
            WHEELtoWALK_FUNCTION_FAST();
        }
        pos_change_count++;
        if(pos_change_count>=1000000)
            pos_change_count=0;

        if(WB_FLAG == true)
        {
            // Global whole body model

            WBmotion->updateAll();
            WBmotion->WBIK();

            for(int i=RHY; i<=LAR; i++) joint->SetJointRefAngle(i, WBmotion->Q_filt_34x1[idRHY+i-RHY]*R2D);

            joint->SetJointRefAngle(WST, WBmotion->Q_filt_34x1[idWST]*R2D);

            joint->SetJointRefAngle(RSP, WBmotion->Q_filt_34x1[idRSP]*R2D);
            joint->SetJointRefAngle(RSR, WBmotion->Q_filt_34x1[idRSR]*R2D - OFFSET_RSR);
            joint->SetJointRefAngle(RSY, WBmotion->Q_filt_34x1[idRSY]*R2D);
            joint->SetJointRefAngle(REB, WBmotion->Q_filt_34x1[idREB]*R2D - OFFSET_ELB);
            joint->SetJointRefAngle(RWY, WBmotion->Q_filt_34x1[idRWY]*R2D);
            joint->SetJointRefAngle(RWP, WBmotion->Q_filt_34x1[idRWP]*R2D);
            joint->SetJointRefAngle(RWY2, WBmotion->Q_filt_34x1[idRWY2]*R2D);

            joint->SetJointRefAngle(LSP, WBmotion->Q_filt_34x1[idLSP]*R2D);
            joint->SetJointRefAngle(LSR, WBmotion->Q_filt_34x1[idLSR]*R2D - OFFSET_LSR);
            joint->SetJointRefAngle(LSY, WBmotion->Q_filt_34x1[idLSY]*R2D);
            joint->SetJointRefAngle(LEB, WBmotion->Q_filt_34x1[idLEB]*R2D - OFFSET_ELB);
            joint->SetJointRefAngle(LWY, WBmotion->Q_filt_34x1[idLWY]*R2D);
            joint->SetJointRefAngle(LWP, WBmotion->Q_filt_34x1[idLWP]*R2D);
            joint->SetJointRefAngle(LWY2, WBmotion->Q_filt_34x1[idLWY2]*R2D);

            if(!CheckMotionOwned())
                WB_FLAG = false;
        }
        // Yaw compensation
        _OW_YAW_VALUE += sharedSEN->IMU[0].YawVel*0.005;
        if(_OW_YAW_VALUE > 360.)
            _OW_YAW_VALUE = 0.;
        else if(_OW_YAW_VALUE <-360.)
            _OW_YAW_VALUE = 0.;

        // Save
//        double totalMy = sharedData->FTMy[0]+sharedData->FTMy[1];
//        double totalFz = sharedData->FTFz[1]+sharedData->FTFz[0];
//        double ZMP_x = -1000.*totalMy/totalFz;
//        if(saveFlag==1)
//        {
//            //DataBuf[0][saveIndex]= sharedData->CurrentPosition[MC_ID_CH_Pairs[RHP].id][MC_ID_CH_Pairs[RHP].ch]*1.;
//            //DataBuf[1][saveIndex]= sharedData->CurrentPosition[MC_ID_CH_Pairs[RKN].id][MC_ID_CH_Pairs[RKN].ch]*1.;
//            //DataBuf[2][saveIndex]= sharedData->CurrentPosition[MC_ID_CH_Pairs[RAP].id][MC_ID_CH_Pairs[RAP].ch]*1.;
//            //DataBuf[3][saveIndex]= ZMP_x;
//            DataBuf[0][saveIndex]=saveIndex;
//            DataBuf[1][saveIndex]=RWHnow;
//            DataBuf[2][saveIndex]=LWHnow;
//            DataBuf[3][saveIndex]=sharedData->FOGRoll;
//            DataBuf[4][saveIndex]=sharedData->FOGPitch;
//            DataBuf[5][saveIndex]=sharedData->FOGYaw;
//            DataBuf[6][saveIndex]=sharedData->IMUAccX[0];
//            DataBuf[7][saveIndex]=sharedData->IMUAccY[0];
//            DataBuf[8][saveIndex]=sharedData->IMUAccZ[0];
//            DataBuf[9][saveIndex]=77;
//            saveIndex++;

//            if(saveIndex==100000)
//                saveIndex=0;
//        }

        joint->MoveAllJoint();
        suspendThread();
    }
}
// --------------------------------------------------------------------------------------------- //
int OW_Operation_in_RealTime(unsigned long ulCount, int motion_number)
{
    int result = 0;
    switch(motion_number)
    {
    case MOTION_1:
        if(_isFirst == 1)
        {
            _OW_YAW_VALUE = 0.;
            _OW_YAW_DES =0.;

            cur_Distance=_path_plan[motion_number][0];//deg
            cur_T1=cur_T3=maxAngularVel/maxAngularAcc;//(s)

            double temp_S_triangle=cur_T1*maxAngularVel;//deg
            if(temp_S_triangle>cur_Distance)
            {
                cur_T1=cur_T3=sqrt(cur_Distance/maxAngularAcc);
                cur_T2=0.;
            }
            else
            {
                double temp_S_remain=cur_Distance-temp_S_triangle;
                cur_T2=temp_S_remain/maxAngularVel;
            }
            cur_T1_count=(unsigned long)(cur_T1*200);//count
            cur_T2_count=(unsigned long)(cur_T2*200);//count
            cur_T3_count=(unsigned long)(cur_T3*200);//count

            gap_end_T1=cur_rest_count+cur_T1_count;
            gap_end_T2=cur_rest_count+cur_T1_count+cur_T2_count;
            gap_end_T3=cur_rest_count+cur_T1_count+cur_T2_count+cur_T3_count;

            _isFirst = 0;
            _localCount=0;
        }
        else
        {
            if(ulCount<=cur_rest_count)
                _GOAL_DELTA=0.;
            else if((ulCount>cur_rest_count)&&(ulCount<=gap_end_T1))
                _GOAL_DELTA=_GOAL_DELTA+maxAngularAcc/200.;
            else if((ulCount>gap_end_T1)&&(ulCount<=gap_end_T2))
                _GOAL_DELTA=maxAngularVel;//(deg/s)
            else if((ulCount>gap_end_T2)&&(ulCount<=gap_end_T3))
                _GOAL_DELTA=_GOAL_DELTA-maxAngularAcc/200.;
            if(ulCount==gap_end_T3)
            {
                _OW_YAW_DES = _OW_YAW_VALUE;

                _GOAL_DELTA=0.;
                _isFirst = 1;
                result = 1;
            }
        }
        break;

    case COMPEN_1:
        if(_isFirst == 1)
        {
            _OW_YAW_VALUE = 0.;
            _OW_YAW_DES =0.;

            cur_Distance=_compen_1[0];//deg
            cur_T1=cur_T3=maxAngularVel/maxAngularAcc;//(s)

            double temp_S_triangle=cur_T1*maxAngularVel;//deg
            if(temp_S_triangle>cur_Distance)
            {
                cur_T1=cur_T3=sqrt(cur_Distance/maxAngularAcc);
                cur_T2=0.;
            }
            else
            {
                double temp_S_remain=cur_Distance-temp_S_triangle;
                cur_T2=temp_S_remain/maxAngularVel;
            }
            cur_T1_count=(unsigned long)(cur_T1*200);//count
            cur_T2_count=(unsigned long)(cur_T2*200);//count
            cur_T3_count=(unsigned long)(cur_T3*200);//count

            gap_end_T1=cur_rest_count+cur_T1_count;
            gap_end_T2=cur_rest_count+cur_T1_count+cur_T2_count;
            gap_end_T3=cur_rest_count+cur_T1_count+cur_T2_count+cur_T3_count;

            _isFirst = 0;
            _localCount=0;
        }
        else
        {
            if(ulCount<=cur_rest_count)
                _GOAL_DELTA=0.;
            else if((ulCount>cur_rest_count)&&(ulCount<=gap_end_T1))
                _GOAL_DELTA=_GOAL_DELTA+maxAngularAcc/200.;
            else if((ulCount>gap_end_T1)&&(ulCount<=gap_end_T2))
                _GOAL_DELTA=maxAngularVel;//(deg/s)
            else if((ulCount>gap_end_T2)&&(ulCount<=gap_end_T3))
                _GOAL_DELTA=_GOAL_DELTA-maxAngularAcc/200.;
            if(ulCount==gap_end_T3)
            {
                _OW_YAW_DES = _OW_YAW_VALUE;

                _GOAL_DELTA=0.;
                _isFirst = 1;
                result = 1;
            }
        }
        break;

    case COMPEN_1_1:
        if(_isFirst == 1)
        {
            cur_Distance=_compen_1_1[0];//deg
            cur_T1=cur_T3=maxAngularVel/maxAngularAcc;//(s)

            double temp_S_triangle=cur_T1*maxAngularVel;//deg
            if(temp_S_triangle>cur_Distance)
            {
                cur_T1=cur_T3=sqrt(cur_Distance/maxAngularAcc);
                cur_T2=0.;
            }
            else
            {
                double temp_S_remain=cur_Distance-temp_S_triangle;
                cur_T2=temp_S_remain/maxAngularVel;
            }
            cur_T1_count=(unsigned long)(cur_T1*200);//count
            cur_T2_count=(unsigned long)(cur_T2*200);//count
            cur_T3_count=(unsigned long)(cur_T3*200);//count

            gap_end_T1=cur_rest_count+cur_T1_count;
            gap_end_T2=cur_rest_count+cur_T1_count+cur_T2_count;
            gap_end_T3=cur_rest_count+cur_T1_count+cur_T2_count+cur_T3_count;

            _isFirst = 0;
            _localCount=0;
        }
        else
        {
            if(ulCount<=cur_rest_count)
                _GOAL_DELTA=0.;
            else if((ulCount>cur_rest_count)&&(ulCount<=gap_end_T1))
                _GOAL_DELTA=_GOAL_DELTA+maxAngularAcc/200.;
            else if((ulCount>gap_end_T1)&&(ulCount<=gap_end_T2))
                _GOAL_DELTA=maxAngularVel;//(deg/s)
            else if((ulCount>gap_end_T2)&&(ulCount<=gap_end_T3))
                _GOAL_DELTA=_GOAL_DELTA-maxAngularAcc/200.;
            if(ulCount==gap_end_T3)
            {
                _GOAL_DELTA=0.;
                _isFirst = 1;
                result = 1;
            }
        }
        break;

    case MOTION_2:
        if(_isFirst == 1)
        {
            _OW_YAW_VALUE = 0.;
            _OW_DIRCT =0.;

            cur_Distance=_path_plan[motion_number][0];//deg
            cur_T1=cur_T3=maxLinearVel/maxLinearAcc;//(s)

            double temp_S_triangle=cur_T1*maxLinearVel;//deg
            if(temp_S_triangle>cur_Distance)
            {
                cur_T1=cur_T3=sqrt(cur_Distance/maxLinearAcc);
                cur_T2=0.;
            }
            else
            {
                double temp_S_remain=cur_Distance-temp_S_triangle;
                cur_T2=temp_S_remain/maxLinearVel;
            }
            cur_T1_count=(unsigned long)(cur_T1*200);//count
            cur_T2_count=(unsigned long)(cur_T2*200);//count
            cur_T3_count=(unsigned long)(cur_T3*200);//count

            gap_end_T1=cur_rest_count+cur_T1_count;
            gap_end_T2=cur_rest_count+cur_T1_count+cur_T2_count;
            gap_end_T3=cur_rest_count+cur_T1_count+cur_T2_count+cur_T3_count;

            _isFirst = 0;
            _localCount=0;

            _OW_CONTROL_RWH = 0.;
            _OW_CONTROL_LWH = 0;
        }
        else
        {
            if(ulCount<=cur_rest_count)
                _GOAL_DELTA=0.;
            else if((ulCount>cur_rest_count)&&(ulCount<=gap_end_T1))
                _GOAL_DELTA=_GOAL_DELTA+maxLinearAcc/200.;
            else if((ulCount>gap_end_T1)&&(ulCount<=gap_end_T2))
                _GOAL_DELTA=maxLinearVel;//(m/s)
            else if((ulCount>gap_end_T2)&&(ulCount<=gap_end_T3))
                _GOAL_DELTA=_GOAL_DELTA-maxLinearAcc/200.;
            if(ulCount==gap_end_T3)
            {
                _OW_CONTROL_RWH = 0.;
                _OW_CONTROL_LWH = 0;
                _OW_DIRCT = _OW_YAW_VALUE;

                _GOAL_DELTA=0.;
                _isFirst = 1;
                result = 1;
            }else{
                double input = sharedSEN->IMU[0].YawVel*0.005;
                double control = -input*D2R*_robot_width/2./_wheel_radi*R2D;
                if(control >0.1)
                    control = 0.1;
                else if(control<-0.1)
                    control = -0.1;

                double cur_RWH = control;
                double cur_LWH = -control;
                double alpha = 0.2;
                _OW_CONTROL_RWH = alpha*_OW_CONTROL_RWH + (1-alpha)*cur_RWH;
                _OW_CONTROL_LWH = alpha*_OW_CONTROL_LWH + (1-alpha)*cur_LWH;
            }
        }
        break;

    case MOTION_3:
        if(_isFirst == 1)
        {
            _OW_YAW_VALUE = 0.;
            _OW_YAW_DES =0.;

            cur_Distance=_path_plan[motion_number][0];//deg
            cur_T1=cur_T3=maxAngularVel/maxAngularAcc;//(s)

            double temp_S_triangle=cur_T1*maxAngularVel;//deg
            if(temp_S_triangle>cur_Distance)
            {
                cur_T1=cur_T3=sqrt(cur_Distance/maxAngularAcc);
                cur_T2=0.;
            }
            else
            {
                double temp_S_remain=cur_Distance-temp_S_triangle;
                cur_T2=temp_S_remain/maxAngularVel;
            }
            cur_T1_count=(unsigned long)(cur_T1*200);//count
            cur_T2_count=(unsigned long)(cur_T2*200);//count
            cur_T3_count=(unsigned long)(cur_T3*200);//count

            gap_end_T1=cur_rest_count+cur_T1_count;
            gap_end_T2=cur_rest_count+cur_T1_count+cur_T2_count;
            gap_end_T3=cur_rest_count+cur_T1_count+cur_T2_count+cur_T3_count;

            _isFirst = 0;
            _localCount=0;
        }
        else
        {
            if(ulCount<=cur_rest_count)
                _GOAL_DELTA=0.;
            else if((ulCount>cur_rest_count)&&(ulCount<=gap_end_T1))
                _GOAL_DELTA=_GOAL_DELTA+maxAngularAcc/200.;
            else if((ulCount>gap_end_T1)&&(ulCount<=gap_end_T2))
                _GOAL_DELTA=maxAngularVel;//(deg/s)
            else if((ulCount>gap_end_T2)&&(ulCount<=gap_end_T3))
                _GOAL_DELTA=_GOAL_DELTA-maxAngularAcc/200.;
            if(ulCount==gap_end_T3)
            {
                _OW_YAW_DES = _OW_YAW_VALUE;

                _GOAL_DELTA=0.;
                _isFirst = 1;
                result = 1;
            }
        }
        break;

    case COMPEN_3:
        if(_isFirst == 1)
        {
            _OW_YAW_VALUE = 0.;
            _OW_YAW_DES =0.;

            cur_Distance=_compen_3[0];//deg
            cur_T1=cur_T3=maxAngularVel/maxAngularAcc;//(s)

            double temp_S_triangle=cur_T1*maxAngularVel;//deg
            if(temp_S_triangle>cur_Distance)
            {
                cur_T1=cur_T3=sqrt(cur_Distance/maxAngularAcc);
                cur_T2=0.;
            }
            else
            {
                double temp_S_remain=cur_Distance-temp_S_triangle;
                cur_T2=temp_S_remain/maxAngularVel;
            }
            cur_T1_count=(unsigned long)(cur_T1*200);//count
            cur_T2_count=(unsigned long)(cur_T2*200);//count
            cur_T3_count=(unsigned long)(cur_T3*200);//count

            gap_end_T1=cur_rest_count+cur_T1_count;
            gap_end_T2=cur_rest_count+cur_T1_count+cur_T2_count;
            gap_end_T3=cur_rest_count+cur_T1_count+cur_T2_count+cur_T3_count;

            _isFirst = 0;
            _localCount=0;
        }
        else
        {
            if(ulCount<=cur_rest_count)
                _GOAL_DELTA=0.;
            else if((ulCount>cur_rest_count)&&(ulCount<=gap_end_T1))
                _GOAL_DELTA=_GOAL_DELTA+maxAngularAcc/200.;
            else if((ulCount>gap_end_T1)&&(ulCount<=gap_end_T2))
                _GOAL_DELTA=maxAngularVel;//(deg/s)
            else if((ulCount>gap_end_T2)&&(ulCount<=gap_end_T3))
                _GOAL_DELTA=_GOAL_DELTA-maxAngularAcc/200.;
            if(ulCount==gap_end_T3)
            {
                _OW_YAW_DES = _OW_YAW_VALUE;

                _GOAL_DELTA=0.;
                _isFirst = 1;
                result = 1;
            }
        }
        break;
    case COMPEN_3_1:
        if(_isFirst == 1)
        {
            cur_Distance=_compen_3_1[0];//deg
            cur_T1=cur_T3=maxAngularVel/maxAngularAcc;//(s)

            double temp_S_triangle=cur_T1*maxAngularVel;//deg
            if(temp_S_triangle>cur_Distance)
            {
                cur_T1=cur_T3=sqrt(cur_Distance/maxAngularAcc);
                cur_T2=0.;
            }
            else
            {
                double temp_S_remain=cur_Distance-temp_S_triangle;
                cur_T2=temp_S_remain/maxAngularVel;
            }
            cur_T1_count=(unsigned long)(cur_T1*200);//count
            cur_T2_count=(unsigned long)(cur_T2*200);//count
            cur_T3_count=(unsigned long)(cur_T3*200);//count

            gap_end_T1=cur_rest_count+cur_T1_count;
            gap_end_T2=cur_rest_count+cur_T1_count+cur_T2_count;
            gap_end_T3=cur_rest_count+cur_T1_count+cur_T2_count+cur_T3_count;

            _isFirst = 0;
            _localCount=0;
        }
        else
        {
            if(ulCount<=cur_rest_count)
                _GOAL_DELTA=0.;
            else if((ulCount>cur_rest_count)&&(ulCount<=gap_end_T1))
                _GOAL_DELTA=_GOAL_DELTA+maxAngularAcc/200.;
            else if((ulCount>gap_end_T1)&&(ulCount<=gap_end_T2))
                _GOAL_DELTA=maxAngularVel;//(deg/s)
            else if((ulCount>gap_end_T2)&&(ulCount<=gap_end_T3))
                _GOAL_DELTA=_GOAL_DELTA-maxAngularAcc/200.;
            if(Tank_Mode_Flag == true)
            {
                if(ulCount==gap_end_T3){
                    _GOAL_DELTA=0.;
                    _isFirst = 1;
                    result = 1;
                }
            }
            else
            {
                if(ulCount==(gap_end_T3)){
                    _GOAL_DELTA=0.;
                    if(_GAIN_HIP_FLAG == true){
                        MCJointGainOverride(0,2,1,0,1000);
                        MCJointGainOverride(1,8,1,0,1000);
                    }
                }
                if(ulCount==(gap_end_T3+200)){
                    _GOAL_DELTA=0.;
                    _isFirst = 1;
                    result = 1;
                    noWaitFlag = false;
                }
            }
        }
        break;
    }

    if(motion_number==MOTION_1 || motion_number==MOTION_3)
    {
        // Rotation part
        double temp_wheel_omega;//deg/s
        temp_wheel_omega=((_robot_width/2.)*_GOAL_DELTA)/_wheel_radi;//deg/s
        if(_path_plan[motion_number][1]>0.){//CCW
            _OW_RIGHT=temp_wheel_omega;
            Tank_delta = Tank_delta - _GOAL_DELTA/200.;
        }
        else{//CW
            _OW_RIGHT=-temp_wheel_omega;
            Tank_delta = Tank_delta + _GOAL_DELTA/200.;
        }
        _OW_LEFT=-_OW_RIGHT;
    }
    else if(motion_number==COMPEN_1 || motion_number==COMPEN_3 || motion_number==COMPEN_1_1 || motion_number==COMPEN_3_1)
    {
        // Rotation part
        double temp_wheel_omega;//deg/s
        temp_wheel_omega=((_robot_width/2.)*_GOAL_DELTA)/_wheel_radi;//deg/s

        double temp_index;
        if(motion_number == COMPEN_1)
            temp_index = _compen_1[1];
        else if(motion_number == COMPEN_3)
            temp_index = _compen_3[1];
        else if(motion_number == COMPEN_1_1)
            temp_index = _compen_1_1[1];
        else if(motion_number == COMPEN_3_1)
            temp_index = _compen_3_1[1];
        else
            temp_index = 1;

        if(temp_index>0.){//CCW
            _OW_RIGHT=temp_wheel_omega;
            Tank_delta = Tank_delta - _GOAL_DELTA/200.;
        }else{//CW
            _OW_RIGHT=-temp_wheel_omega;
            Tank_delta = Tank_delta + _GOAL_DELTA/200.;
        }
        _OW_LEFT=-_OW_RIGHT;
    }
    else if(motion_number==MOTION_2)
    {
        // Direct part
        _OW_RIGHT=_path_plan[1][1]*_GOAL_DELTA/_wheel_radi*R2D;
        _OW_LEFT=_OW_RIGHT;

        Tank_delta = Tank_delta ;
    }
    else
    {
        _OW_RIGHT=0.;
        _OW_LEFT=0.;
    }

    return result;
}
int OW_Operation_in_RealTime_Radius(unsigned long ulCount, int motion_number)
{
    int result=0;

    switch(motion_number)
    {
    case RADIUS_1:
        if(_isFirst == 1)
        {
            _OW_YAW_VALUE = 0.;
            _OW_YAW_DES =0.;

            cur_Distance=WMR_angle;//deg
            cur_T1=cur_T3=WMR_omega_global_max/WMR_alpha_global_max;//(s)

            double temp_S_triangle=cur_T1*WMR_omega_global_max;//deg
            if(temp_S_triangle>cur_Distance)
            {
                cur_T1=cur_T3=sqrt(cur_Distance/WMR_alpha_global_max);
                cur_T2=0.;
            }
            else
            {
                double temp_S_remain=cur_Distance-temp_S_triangle;
                cur_T2=temp_S_remain/WMR_omega_global_max;
            }
            cur_T1_count=(unsigned long)(cur_T1*200);//count
            cur_T2_count=(unsigned long)(cur_T2*200);//count
            cur_T3_count=(unsigned long)(cur_T3*200);//count

            gap_end_T1=cur_rest_count+cur_T1_count;
            gap_end_T2=cur_rest_count+cur_T1_count+cur_T2_count;
            gap_end_T3=cur_rest_count+cur_T1_count+cur_T2_count+cur_T3_count;

            _isFirst = 0;
            _localCount=0;
        }
        else
        {
            if(ulCount<=cur_rest_count)
                _GOAL_DELTA=0.;
            else if((ulCount>cur_rest_count)&&(ulCount<=gap_end_T1))
                _GOAL_DELTA=_GOAL_DELTA+WMR_alpha_global_max/200.;
            else if((ulCount>gap_end_T1)&&(ulCount<=gap_end_T2))
                _GOAL_DELTA=WMR_omega_global_max;//(deg/s)
            else if((ulCount>gap_end_T2)&&(ulCount<=gap_end_T3))
                _GOAL_DELTA=_GOAL_DELTA-WMR_alpha_global_max/200.;
            if(ulCount==(gap_end_T3)){
                _GOAL_DELTA=0.;
            }
            if(ulCount==(gap_end_T3))
            {
                _OW_YAW_DES = _OW_YAW_VALUE;

                _GOAL_DELTA=0.;
                _isFirst = 1;
                result = 1;
            }
        }
        break;
    case RADIUS_2:
        if(_isFirst == 1)
        {
            _OW_YAW_VALUE = 0.;
            _OW_YAW_DES =0.;

            cur_Distance = fabs(_r_compen_1[0]);//deg
            cur_T1=cur_T3=WMR_omega_global_max/WMR_alpha_global_max;//(s)

            double temp_S_triangle=cur_T1*WMR_omega_global_max;//deg
            if(temp_S_triangle>cur_Distance)
            {
                cur_T1=cur_T3=sqrt(cur_Distance/WMR_alpha_global_max);
                cur_T2=0.;
            }
            else
            {
                double temp_S_remain=cur_Distance-temp_S_triangle;
                cur_T2=temp_S_remain/WMR_omega_global_max;
            }
            cur_T1_count=(unsigned long)(cur_T1*200);//count
            cur_T2_count=(unsigned long)(cur_T2*200);//count
            cur_T3_count=(unsigned long)(cur_T3*200);//count

            gap_end_T1=cur_rest_count+cur_T1_count;
            gap_end_T2=cur_rest_count+cur_T1_count+cur_T2_count;
            gap_end_T3=cur_rest_count+cur_T1_count+cur_T2_count+cur_T3_count;

            _isFirst = 0;
            _localCount=0;
        }
        else
        {
            if(ulCount<=cur_rest_count)
                _GOAL_DELTA=0.;
            else if((ulCount>cur_rest_count)&&(ulCount<=gap_end_T1))
                _GOAL_DELTA=_GOAL_DELTA+WMR_alpha_global_max/200.;
            else if((ulCount>gap_end_T1)&&(ulCount<=gap_end_T2))
                _GOAL_DELTA=WMR_omega_global_max;//(deg/s)
            else if((ulCount>gap_end_T2)&&(ulCount<=gap_end_T3))
                _GOAL_DELTA=_GOAL_DELTA-WMR_alpha_global_max/200.;
            if(ulCount==(gap_end_T3)){
                _GOAL_DELTA=0.;
            }
            if(ulCount==(gap_end_T3))
            {
                _OW_YAW_DES = _OW_YAW_VALUE;

                _GOAL_DELTA=0.;
                _isFirst = 1;
                result = 1;
            }
        }
        break;
    case RADIUS_3:
        if(_isFirst == 1)
        {
            _OW_YAW_VALUE = 0.;
            _OW_YAW_DES =0.;

            cur_Distance = fabs(_r_compen_2[0]);//deg
            cur_T1=cur_T3=WMR_omega_global_max/WMR_alpha_global_max;//(s)

            double temp_S_triangle=cur_T1*WMR_omega_global_max;//deg
            if(temp_S_triangle>cur_Distance)
            {
                cur_T1=cur_T3=sqrt(cur_Distance/WMR_alpha_global_max);
                cur_T2=0.;
            }
            else
            {
                double temp_S_remain=cur_Distance-temp_S_triangle;
                cur_T2=temp_S_remain/WMR_omega_global_max;
            }
            cur_T1_count=(unsigned long)(cur_T1*200);//count
            cur_T2_count=(unsigned long)(cur_T2*200);//count
            cur_T3_count=(unsigned long)(cur_T3*200);//count

            gap_end_T1=cur_rest_count+cur_T1_count;
            gap_end_T2=cur_rest_count+cur_T1_count+cur_T2_count;
            gap_end_T3=cur_rest_count+cur_T1_count+cur_T2_count+cur_T3_count;

            _isFirst = 0;
            _localCount=0;
        }
        else
        {
            if(ulCount<=cur_rest_count)
                _GOAL_DELTA=0.;
            else if((ulCount>cur_rest_count)&&(ulCount<=gap_end_T1))
                _GOAL_DELTA=_GOAL_DELTA+WMR_alpha_global_max/200.;
            else if((ulCount>gap_end_T1)&&(ulCount<=gap_end_T2))
                _GOAL_DELTA=WMR_omega_global_max;//(deg/s)
            else if((ulCount>gap_end_T2)&&(ulCount<=gap_end_T3))
                _GOAL_DELTA=_GOAL_DELTA-WMR_alpha_global_max/200.;
            if(ulCount==(gap_end_T3)){
                _GOAL_DELTA=0.;
            }
            if(ulCount==(gap_end_T3))
            {
                _OW_YAW_DES = _OW_YAW_VALUE;

                _GOAL_DELTA=0.;
                _isFirst = 1;
                result = 1;
            }
        }
        break;
    }

    // move wheel
    double temp_fb=1;
    if(motion_number == RADIUS_1)
        temp_fb = (double)WMR_fb;
    else if(motion_number == RADIUS_2)
        temp_fb = _r_compen_1[1];
    else if(motion_number == RADIUS_3)
        temp_fb = _r_compen_2[1];

    double fast_wheel=temp_fb*_GOAL_DELTA*D2R*(WMR_radius+0.5*_robot_width)/_wheel_radi*R2D;
    double slow_wheel=temp_fb*_GOAL_DELTA*D2R*(WMR_radius-0.5*_robot_width)/_wheel_radi*R2D;
    if(WMR_dir==1)//right
    {
        _OW_RIGHT = slow_wheel;
        _OW_LEFT = fast_wheel;
    }
    else//left
    {
        _OW_RIGHT = fast_wheel;
        _OW_LEFT = slow_wheel;
    }

    return result;
}
int OW_WHEEL_MOVEMENT(void)
{
    // Auto trajactory mode
    //printf("Wheel Output   %f   %f\n",_OW_RIGHT, _OW_LEFT);
    double OW_RWH_VEL, OW_LWH_VEL;//deg/unit_step
    if(Control_ON == false){
        OW_RWH_VEL=_OW_RIGHT/200.;
        OW_LWH_VEL=_OW_LEFT/200.;
    }else if(Control_ON == true){
        OW_RWH_VEL=_OW_RIGHT/200. + _OW_CONTROL_RWH;
        OW_LWH_VEL=_OW_LEFT/200.  + _OW_CONTROL_LWH;
    }
    joint->SetMoveJoint(RWH, OW_RWH_VEL, 5, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWH, OW_LWH_VEL, 5, MOVE_ABSOLUTE);
    return 0;
}
// --------------------------------------------------------------------------------------------- //
void *RBFlagThread(void *)
{
    while(isTerminated == 0)
    {
        usleep(300);
        //if(HasAnyOwnership()){
            if(sharedCMD->SYNC_SIGNAL[PODO_NO] == true){
                joint->JointUpdate();
                resumeThread();
            }
        //}
    }
}

// --------------------------------------------------------------------------------------------- //
int CheckMotionOwned()
{
    for(int i=0;i<NO_OF_JOINTS;i++)
    {
        if(sharedCMD->MotionOwner[MC_ID_CH_Pairs[i].id][MC_ID_CH_Pairs[i].ch]!=PODO_NO)	return 0;
    }
    return 1;
}

// --------------------------------------------------------------------------------------------- //
int HasAnyOwnership(){
    for(int i=0; i<NO_OF_JOINTS; i++){
        if(sharedCMD->MotionOwner[MC_ID_CH_Pairs[i].id][MC_ID_CH_Pairs[i].ch] == PODO_NO)
            return true;
    }
    return false;
}
// --------------------------------------------------------------------------------------------- //
void CatchSignals(int _signal)
{
    switch(_signal){
    case SIGHUP:
    case SIGINT:     // Ctrl-c
    case SIGTERM:    // "kill" from shell
    case SIGKILL:
        isTerminated = -1;
        break;
    }
    usleep(1000*500);
}
// --------------------------------------------------------------------------------------------- //

// --------------------------------------------------------------------------------------------- //
int RBInitialize(void)
{
    // Block program termination
    isTerminated = 0;

    int shmFD;
    // Core Shared Memory Creation [Reference]==================================
    shmFD = shm_open(RBCORE_SHM_NAME_REFERENCE, O_RDWR, 0666);
    if(shmFD == -1){
        FILE_LOG(logERROR) << "Fail to open core shared memory [Reference]";
        return false;
    }else{
        if(ftruncate(shmFD, sizeof(RBCORE_SHM_REFERENCE)) == -1){
            FILE_LOG(logERROR) << "Fail to truncate core shared memory [Reference]";
            return false;
        }else{
            sharedREF = (pRBCORE_SHM_REFERENCE)mmap(0, sizeof(RBCORE_SHM_REFERENCE), PROT_WRITE, MAP_SHARED, shmFD, 0);
            if(sharedREF == (void*)-1){
                FILE_LOG(logERROR) << "Fail to mapping core shared memory [Reference]";
                return false;
            }
        }
    }
    FILE_LOG(logSUCCESS) << "Core shared memory creation = OK [Reference]";
    // =========================================================================

    // Core Shared Memory Creation [Sensor]=====================================
    shmFD = shm_open(RBCORE_SHM_NAME_SENSOR, O_RDWR, 0666);
    if(shmFD == -1){
        FILE_LOG(logERROR) << "Fail to open core shared memory [Sensor]";
        return false;
    }else{
        if(ftruncate(shmFD, sizeof(RBCORE_SHM_SENSOR)) == -1){
            FILE_LOG(logERROR) << "Fail to truncate core shared memory [Sensor]";
            return false;
        }else{
            sharedSEN = (pRBCORE_SHM_SENSOR)mmap(0, sizeof(RBCORE_SHM_SENSOR), PROT_READ, MAP_SHARED, shmFD, 0);
            if(sharedSEN == (void*)-1){
                FILE_LOG(logERROR) << "Fail to mapping core shared memory [Sensor]";
                return false;
            }
        }
    }
    FILE_LOG(logSUCCESS) << "Core shared memory creation = OK [Sensor]";
    // =========================================================================

    // Core Shared Memory Creation [Command]====================================
    shmFD = shm_open(RBCORE_SHM_NAME_COMMAND, O_RDWR, 0666);
    if(shmFD == -1){
        FILE_LOG(logERROR) << "Fail to open core shared memory [Command]";
        return false;
    }else{
        if(ftruncate(shmFD, sizeof(RBCORE_SHM_COMMAND)) == -1){
            FILE_LOG(logERROR) << "Fail to truncate core shared memory [Command]";
            return false;
        }else{
            sharedCMD = (pRBCORE_SHM_COMMAND)mmap(0, sizeof(RBCORE_SHM_COMMAND), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
            if(sharedCMD == (void*)-1){
                FILE_LOG(logERROR) << "Fail to mapping core shared memory [Command]";
                return false;
            }
        }
    }
    FILE_LOG(logSUCCESS) << "Core shared memory creation = OK [Command]";
    // =========================================================================


    // User Shared Memory Creation ============================================
    shmFD = shm_open(USER_SHM_NAME, O_RDWR, 0666);
    if(shmFD == -1){
        FILE_LOG(logERROR) << "Fail to open user shared memory";
        return -1;
    }else{
        if(ftruncate(shmFD, sizeof(USER_SHM)) == -1){
            FILE_LOG(logERROR) << "Fail to truncate user shared memory";
            return -1;
        }else{
            userData = (pUSER_SHM)mmap(0, sizeof(USER_SHM), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
            if(userData == (void*)-1){
                FILE_LOG(logERROR) << "Fail to mapping user shared memory";
                return -1;
            }
        }
    }
    FILE_LOG(logSUCCESS) << "User shared memory creation = OK";
    // =========================================================================


    // Initialize internal joint classes =======================================
    joint = new JointControlClass(sharedREF, sharedSEN, sharedCMD, PODO_NO);
    joint->RefreshToCurrentReference();
    // =========================================================================


    // Create and start real-time thread =======================================
    int threadID;
    threadID= pthread_create(&rtFlagCon, NULL, &RBFlagThread, NULL);
    if(threadID < 0){
        FILE_LOG(logERROR) << "Fail to create Flag real-time thread";
        return -1;
    }
    threadID= pthread_create(&rtTaskCon, NULL, &RBTaskThread, NULL);
    if(threadID < 0){
        FILE_LOG(logERROR) << "Fail to create Task real-time thread";
        return -1;
    }
    // =========================================================================

    return 0;
}
//int	PushCANMessage(MANUAL_CAN MCData){
//    for(int i=0; i<MAX_MANUAL_CAN; i++){
//        if(sharedData->ManualCAN[i].status == MANUALCAN_EMPTY){
//            sharedData->ManualCAN[i].status = MANUALCAN_WRITING;
//            sharedData->ManualCAN[i].channel = MCData.channel;
//            sharedData->ManualCAN[i].id = MCData.id;
//            sharedData->ManualCAN[i].dlc = MCData.dlc;
//            for(int j=0; j<MCData.dlc; j++){
//                sharedData->ManualCAN[i].data[j] = MCData.data[j];
//            }
//            sharedData->ManualCAN[i].status = MANUALCAN_NEW;
//            return RB_SUCCESS;
//        }
//    }
//    cout << "Fail to send Manual CAN..!!" << endl;
//    return RB_FAIL;
//}
//int RBJointGainOverride(unsigned int _canch, unsigned int _bno, unsigned int _override1, unsigned int _override2, unsigned int _duration){
//    // MsgID                Byte0	Byte1	Byte2	Byte3	Byte4		Byte5
//    // CANID_SEND_CMD		BNO		0x6F	OVER1	OVER2	DURATION	DURATION
//    MANUAL_CAN	MCData;

//    MCData.channel = _canch;
//    MCData.id = COMMAND_CANID;
//    MCData.dlc = 6;
//    MCData.data[0] = _bno;
//    MCData.data[1] = 0x6F;
//    MCData.data[2] = _override1;
//    MCData.data[3] = _override2;
//    MCData.data[4] = (_duration & 0xFF);
//    MCData.data[5] = ((_duration>>8) & (0xFF));

//    return PushCANMessage(MCData);
//}

//int RBBoardSetSwitchingMode(unsigned int _canch, unsigned int _bno, int _mode){
//    // MsgID		Byte0	Byte1	Byte2
//    // CMD_TXDF		BNO		0x13	_mode
//    MANUAL_CAN	MCData;

//    MCData.channel = _canch;
//    MCData.id = COMMAND_CANID;
//    MCData.dlc = 3;
//    MCData.data[0] = _bno;
//    MCData.data[1] = 0x13;
//    MCData.data[2] = _mode;

//    return PushCANMessage(MCData);
//}

//int RBenableFrictionCompensation(unsigned int _canch, unsigned int _bno, unsigned int _enable1, unsigned int _enable2){
//    // MsgID		Byte0	Byte1	Byte2
//    // CMD_TXDF		BNO		0xB1	ENABLE
//    MANUAL_CAN	MCData;

//    MCData.channel = _canch;
//    MCData.id = COMMAND_CANID;
//    MCData.dlc = 3;
//    MCData.data[0] = _bno;
//    MCData.data[1] = 0xB1;
//    MCData.data[2] = (_enable1&0x01) | ((_enable2&0x01)<<1);

//    return PushCANMessage(MCData);
//}
// --------------------------------------------------------------------------------------------- //

unsigned int OmniPathGenerator(const double des_x, const double des_y, const double des_a, char _mode)
{
    double theta1=0.;
    double theta2=0.;
    double temp_sign_1, temp_sign_2;
    double line_length=sqrt(fabs(des_x)*fabs(des_x)+fabs(des_y)*fabs(des_y));
    double dir_sign=1;
    if(_mode==0)//direct movement
    {
        dir_sign=1;
        theta1=atan2(des_y,des_x)*R2D;
        theta2=OW_angle_regul(des_a-theta1);
    }
    else if(_mode==1)//Effective movement;
    {
        double temp_ang=atan2(des_y,des_x)*R2D;
        if(temp_ang>90.)
        {
            theta1=temp_ang-180.;
            dir_sign=-1;
        }
        else if(temp_ang<-90.)
        {
            theta1=temp_ang+180.;
            dir_sign=-1;
        }
        else
            theta1=temp_ang;
        theta2=OW_angle_regul(des_a-theta1);
    }
    else
        printf("OW - Wrong mode input...!!!\n");

    if(theta1<0.)
        temp_sign_1=-1.;//right
    else
        temp_sign_1=1.;//left
    if(theta2<0.)
        temp_sign_2=-1.;//right
    else
        temp_sign_2=1.;//left
    _path_plan[0][0]=fabs(theta1);        _path_plan[0][1]=temp_sign_1;
    _path_plan[1][0]=line_length;         _path_plan[1][1]=dir_sign;
    _path_plan[2][0]=fabs(theta2);        _path_plan[2][1]=temp_sign_2;

    printf("Path planned Data-----------------\n");
    printf("Scene1: %f   %f\n", _path_plan[0][0], _path_plan[0][1]);
    printf("Scene2: %f   %f\n", _path_plan[1][0], _path_plan[1][1]);
    printf("Scene3: %f   %f\n", _path_plan[2][0], _path_plan[2][1]);
    printf("----------------------------------\n");
    printf("\n");

    return 0;
}

double OW_angle_regul(const double input_angle)
{
    double result_angle=0;
    if(input_angle<-180.)
        result_angle=input_angle+360.;
    else if(input_angle>180)
        result_angle=input_angle-360.;
    else
        result_angle=input_angle;
    return result_angle;
}


// --------------------------------------------------------------------------------------------- //
// --------------------------------------------------------------------------------------------- //
// Task mode functions
// --------------------------------------------------------------------------------------------- //
// --------------------------------------------------------------------------------------------- //

void StartWBIKmotion(int _mode)
{
    WB_FLAG = false;
    usleep(10*1000);

    joint->RefreshToCurrentReference();

    WBmotion->ResetGlobalCoord(_mode);

    WBmotion->StopAll();

    WBmotion->RefreshToCurrentReference();

    joint->SetAllMotionOwner();

    WB_FLAG = true;
}
/**************************************************************************************************
 *File Save
 *************************************************************************************************/
void SaveFile(void)
{
    FILE* fp;
    unsigned int i, j;
    fp = fopen("OmniWheel.txt", "w");

    for(i=0 ; i<saveIndex ; i++)
    {
        for(j=0 ; j<10 ; j++)
            fprintf(fp, "%f\t", DataBuf[j][i]);
        fprintf(fp, "\n");
    }
    fclose(fp);

    saveIndex=0;
    saveFlag=0;
}
void WALKtoWHEEL_FUNCTION_FAST(void)
{
    if(pos_change_count == 2){
        StartWBIKmotion(-1);
    }
    if(pos_change_count == 5){
        _PEL_Z_WALKREADY=WBmotion->pPelZ;
        _FOOT_CENT_X=(WBmotion->pRF_3x1[0]+WBmotion->pLF_3x1[0])/2.;
        _COM_X_WALKREADY=WBmotion->pCOM_2x1[0];
        printf("PEL Z::::: %lf\n",_PEL_Z_WALKREADY);
    }
    if(pos_change_count == 7){
        quat qt_q_pelv=quat(vec3(0,1,0),-45*D2R);
        doubles ds_q_pelv(4);
        for(int k=0;k<4;k++)
            ds_q_pelv[k]=qt_q_pelv[k];
        WBmotion->addPELPosInfo(0.395, 3);
        WBmotion->addCOMInfo(_FOOT_CENT_X+0.02, 0., 3);
        WBmotion->addPELOriInfo(ds_q_pelv, 3);
    }
    if(pos_change_count == 608){
        MCsetFrictionParameter(0,4,2,1000,_GAIN_RWH,0);
        MCsetFrictionParameter(1,10,2,1000,_GAIN_LWH,0);
        MCBoardSetSwitchingMode(0,4,1);
        MCBoardSetSwitchingMode(1,10,1);
        MCenableFrictionCompensation(0,4,2,ENABLE);
        MCenableFrictionCompensation(1,10,2,ENABLE);
        MCJointGainOverride(0,4,2,100,1);
        MCJointGainOverride(1,10,2,100,1);
    }
    if(pos_change_count == 612){
        quat qt_q_pelv=quat(vec3(0,1,0), 10*D2R);
        doubles ds_q_pelv(4);
        for(int k=0;k<4;k++)
            ds_q_pelv[k]=qt_q_pelv[k];
        WBmotion->addPELPosInfo(0.358, 2.2);
        WBmotion->addCOMInfo(_FOOT_CENT_X+0.155, 0., 2.2);
        WBmotion->addPELOriInfo(ds_q_pelv, 2.2);
    }
    if(pos_change_count == 1055){
        WB_FLAG = false;
    }
    if(pos_change_count == 1060){
        joint->SetMoveJoint(RAP, _POS_ANKLE_PITCH, 6000, MOVE_ABSOLUTE);
        joint->SetMoveJoint(LAP, _POS_ANKLE_PITCH, 6000, MOVE_ABSOLUTE);
        joint->SetMoveJoint(RKN, 145.0, 6000, MOVE_ABSOLUTE);
        joint->SetMoveJoint(LKN, 145.0, 6000, MOVE_ABSOLUTE);

        joint->SetMoveJoint(RHP, -57.0, 750, MOVE_ABSOLUTE);//a
        joint->SetMoveJoint(LHP, -57.0, 750, MOVE_ABSOLUTE);//a
    }
    if(pos_change_count == 1210){
        joint->SetMoveJoint(RHP, -30., 1880, MOVE_ABSOLUTE);//c
        joint->SetMoveJoint(LHP, -30., 1880, MOVE_ABSOLUTE);//c
    }
    if(pos_change_count == 2262){
        //JS
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[0] =100;//id
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[1] =1;//
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[2] =1;//ch
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_COMMAND= INIT_FIND_POS;

        FILE_LOG(logERROR) << "WHEEL HOME";
        MCJointFindHome(0, MC_GetID(RWH), MC_GetCH(RWH)+1);
        MCJointFindHome(1, MC_GetID(LWH), MC_GetCH(LWH)+1);
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[0] =4;//id
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[1] =1;//
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[2] =1;//ch
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_COMMAND= INIT_FIND_POS;
    }
//    if(pos_change_count == 2282){
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[0] =10;//id
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[1] =1;//
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[2] =1;//ch
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_COMMAND= INIT_FIND_POS;
//    }
    if(pos_change_count == 2302){
        joint->SetMoveJoint(RHP, _POS_HIP_PITCH, 2500, MOVE_ABSOLUTE);// equilibrium point
        joint->SetMoveJoint(LHP, _POS_HIP_PITCH, 2500, MOVE_ABSOLUTE);// equilibrium point

        MCenableFrictionCompensation(0,4,2,DISABLE);
        MCenableFrictionCompensation(1,10,2,DISABLE);
        MCBoardSetSwitchingMode(0,4,0);
        MCBoardSetSwitchingMode(1,10,0);
        MCJointGainOverride(0,4,2,0, 2500);
        MCJointGainOverride(1,10,2,0, 2500);

        if(REAL_MODE == true){
            MCJointGainOverride(0, 3, 1, 35, 330);
            MCJointGainOverride(1, 9, 1, 35, 330);
        }
    }

    if(REAL_MODE == true){
        if(pos_change_count == 2368){
            MCJointGainOverride(0, 3, 1, 100, 2400);
            MCJointGainOverride(1, 9, 1, 100, 2400);
        }
        if(pos_change_count == 2850){
            printf("Debug---------------------------->> Walk 2 Wheel End...!!!\n");
            STATUS_FLAG=STATUS_IDLE;
            WALKtoWHEEL_FLAG=false;
            WB_FLAG = false;
            pos_change_count=0;
        }
    }else{
        if(pos_change_count == 2805){
            printf("Debug---------------------------->> Walk 2 Wheel End...!!!\n");
            STATUS_FLAG=STATUS_IDLE;
            WALKtoWHEEL_FLAG=false;
            WB_FLAG = false;
            pos_change_count=0;
        }
    }
}

void WALKtoWHEEL_FUNCTION(void)
{

    double step1_postime=3.0;//s
    double step2_postime=2.5;//s
    double step3_postime1=8000;//ms
    double step3_postime2=1000;//ms
    double gain_backtime=3000;//ms

    unsigned long step1_start_count=10;
    unsigned long gainover_start_count=step1_start_count+((unsigned long)(step1_postime*200));
    unsigned long step2_start_count=gainover_start_count+30;
    unsigned long step3_start_count_1=step2_start_count+((unsigned long)(step2_postime*200))+50;
    unsigned long step3_start_count_2=step3_start_count_1+((unsigned long)(step3_postime2/5.));
    unsigned long gainback_start_count=step3_start_count_1+((unsigned long)(step3_postime1/5.))+50;
    unsigned long terminal_start_count=gainback_start_count+((unsigned long)(gain_backtime/5.));

    // Reset Global
    if(pos_change_count == 1)
        ;
        ////sharedData->STATE_COMMAND = TCMD_WHEEL_POS_CHANGE_WALK2WH_START;

    if(pos_change_count==3)
    {
        StartWBIKmotion(-1);
    }
    if(pos_change_count==7)
    {
        _PEL_Z_WALKREADY=WBmotion->pPelZ;
        _FOOT_CENT_X=(WBmotion->pRF_3x1[0]+WBmotion->pLF_3x1[0])/2.;
        _COM_X_WALKREADY=WBmotion->pCOM_2x1[0];
        printf("PEL Z::::: %lf\n",_PEL_Z_WALKREADY);
    }
    // Start step 1
    if(pos_change_count==step1_start_count)
    {
        quat qt_q_pelv=quat(vec3(0,1,0),-45*D2R);
        doubles ds_q_pelv(4);
        for(int k=0;k<4;k++)
            ds_q_pelv[k]=qt_q_pelv[k];
        WBmotion->addPELPosInfo(0.395,step1_postime);
        WBmotion->addCOMInfo(_FOOT_CENT_X+0.02, 0., step1_postime);
        WBmotion->addPELOriInfo(ds_q_pelv, step1_postime);
    }
    // Start Over
    if(pos_change_count==gainover_start_count)
    {
        MCsetFrictionParameter(0,4,2,1000,_GAIN_RWH,0);
        MCsetFrictionParameter(1,10,2,1000,_GAIN_LWH,0);
        MCBoardSetSwitchingMode(0,4,1);
        MCBoardSetSwitchingMode(1,10,1);
        MCenableFrictionCompensation(0,4,2,ENABLE);
        MCenableFrictionCompensation(1,10,2,ENABLE);
        MCJointGainOverride(0,4,2,100,1);
        MCJointGainOverride(1,10,2,100,1);
    }    

    // Start step 2
    if(pos_change_count==step2_start_count)
    {
        quat qt_q_pelv=quat(vec3(0,1,0), 10*D2R);
        doubles ds_q_pelv(4);
        for(int k=0;k<4;k++)
            ds_q_pelv[k]=qt_q_pelv[k];
        WBmotion->addPELPosInfo(0.358,step2_postime);
        WBmotion->addCOMInfo(_FOOT_CENT_X+0.155, 0., step2_postime);
        WBmotion->addPELOriInfo(ds_q_pelv, step2_postime);
    }

    //b

    // Goto step 3
    if(pos_change_count==step3_start_count_1)
    {
        WB_FLAG=false;

        joint->SetMoveJoint(RAP, _POS_ANKLE_PITCH, step3_postime1, MOVE_ABSOLUTE);
        joint->SetMoveJoint(LAP, _POS_ANKLE_PITCH, step3_postime1, MOVE_ABSOLUTE);
        joint->SetMoveJoint(RKN, 145.0, step3_postime1, MOVE_ABSOLUTE);
        joint->SetMoveJoint(LKN, 145.0, step3_postime1, MOVE_ABSOLUTE);

        joint->SetMoveJoint(RHP, -57.0, step3_postime2, MOVE_ABSOLUTE);//a
        joint->SetMoveJoint(LHP, -57.0, step3_postime2, MOVE_ABSOLUTE);//a
    }
    if(pos_change_count==step3_start_count_2)
    {
        joint->SetMoveJoint(RHP, -30., step3_postime1-step3_postime2-4500, MOVE_ABSOLUTE);//c
        joint->SetMoveJoint(LHP, -30., step3_postime1-step3_postime2-4500, MOVE_ABSOLUTE);//c
    }
    // Gain back
    if(pos_change_count==gainback_start_count)
    {
        // JS
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[0] =4;//id
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[1] =1;//
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[2] =1;//ch
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_COMMAND= INIT_FIND_POS;

        MCJointFindHome(0, MC_GetID(RWH), MC_GetCH(RWH)+1);
    }
    if(pos_change_count==(gainback_start_count+20))
    {
        // JS
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[0] =10;//id
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[1] =1;//
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[2] =1;//ch
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_COMMAND= INIT_FIND_POS;
        MCJointFindHome(1, MC_GetID(LWH), MC_GetCH(LWH)+1);
    }
    if(pos_change_count==(gainback_start_count+40))
    {
        joint->SetMoveJoint(RHP, _POS_HIP_PITCH, gain_backtime, MOVE_ABSOLUTE);// equilibrium point
        joint->SetMoveJoint(LHP, _POS_HIP_PITCH, gain_backtime, MOVE_ABSOLUTE);// equilibrium point

        MCenableFrictionCompensation(0,4,2,DISABLE);
        MCenableFrictionCompensation(1,10,2,DISABLE);
        MCBoardSetSwitchingMode(0,4,0);
        MCBoardSetSwitchingMode(1,10,0);
        MCJointGainOverride(0,4,2,0,gain_backtime);
        MCJointGainOverride(1,10,2,0,gain_backtime);
    }
    // Terminal
    if(pos_change_count==(terminal_start_count+50))
    {
        ////sharedData->STATE_COMMAND = TCMD_WHEEL_POS_CHANGE_WALK2WH_DONE;
        printf("Debug---------------------------->> Walk 2 Wheel End...!!!\n");
        STATUS_FLAG=STATUS_IDLE;
        WALKtoWHEEL_FLAG=false;
        WB_FLAG = false;
        pos_change_count=0;
    }

}

void WHEELtoWALK_FUNCTION_FAST(void)
{
    if(pos_change_count == 1){
        WB_FLAG = false;
    }
    if(pos_change_count == 2){
        MCsetFrictionParameter(0,4,2,1000,_GAIN_RWH,0);
        MCsetFrictionParameter(1,10,2,1000,_GAIN_LWH,0);
        MCBoardSetSwitchingMode(0,4,1);
        MCBoardSetSwitchingMode(1,10,1);
        MCenableFrictionCompensation(0,4,2,ENABLE);
        MCenableFrictionCompensation(1,10,2,ENABLE);
        MCJointGainOverride(0,4,2,100,1);
        MCJointGainOverride(1,10,2,100,1);
    }
    if(pos_change_count == 5){
        joint->SetMoveJoint(RHP, -30., 1300, MOVE_ABSOLUTE);//c
        joint->SetMoveJoint(LHP, -30., 1300, MOVE_ABSOLUTE);//c
    }
    if(pos_change_count == 270){
        joint->SetMoveJoint(RAP, -94.64, 6000, MOVE_ABSOLUTE);//b
        joint->SetMoveJoint(LAP, -94.64, 6000, MOVE_ABSOLUTE);//b
        joint->SetMoveJoint(RKN, 143.9, 6000, MOVE_ABSOLUTE);//b//141
        joint->SetMoveJoint(LKN, 143.9, 6000, MOVE_ABSOLUTE);//b
    }
    if(pos_change_count == 945){
        joint->SetMoveJoint(RHP, -57.0, 1875, MOVE_ABSOLUTE);//a
        joint->SetMoveJoint(LHP, -57.0, 1875, MOVE_ABSOLUTE);//a
    }
    if(pos_change_count == 1320){
        joint->SetMoveJoint(RHP, -59.32, 750, MOVE_ABSOLUTE);//b//-55
        joint->SetMoveJoint(LHP, -59.32, 750, MOVE_ABSOLUTE);//b
    }
    if(pos_change_count == 1472){
        StartWBIKmotion(-1);
    }
    if(pos_change_count == 1475){
        _FOOT_CENT_X2 =(WBmotion->pRF_3x1[0]+WBmotion->pLF_3x1[0])/2.;
    }
    if(pos_change_count == 1477){
        quat qt_q_pelv=quat(vec3(0,1,0),-45*D2R);
        doubles ds_q_pelv(4);
        for(int k=0;k<4;k++)
            ds_q_pelv[k]=qt_q_pelv[k];
        WBmotion->addPELPosInfo(0.395,2.2);
        WBmotion->addCOMInfo(_FOOT_CENT_X2+0.02, 0., 2.2);
        WBmotion->addPELOriInfo(ds_q_pelv, 2.2);
    }
    if(pos_change_count == 1920){
        quat qt_q_pelv=quat(vec3(0,1,0),0*D2R);
        doubles ds_q_pelv(4);
        for(int k=0;k<4;k++)
            ds_q_pelv[k]=qt_q_pelv[k];
        WBmotion->addPELPosInfo(_PEL_Z_WALKREADY,3.);
        WBmotion->addCOMInfo(_FOOT_CENT_X2+0.01, 0., 3.);
        WBmotion->addPELOriInfo(ds_q_pelv, 3.);
    }
    if(pos_change_count == 2522){
        WB_FLAG = false;
    }
    if(pos_change_count == 2525){
        Wheel_Walk_Ready(1800);
    }

    if(pos_change_count == 2735){
        //JS
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[0] =100;//id
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[1] =1;//
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[2] =1;//ch
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_COMMAND= INIT_FIND_POS;

        MCJointFindHome(0, MC_GetID(RWH), MC_GetCH(RWH)+1);
        MCJointFindHome(1, MC_GetID(LWH), MC_GetCH(LWH)+1);

//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[0] =4;//id
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[1] =1;//
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[2] =1;//ch
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_COMMAND= INIT_FIND_POS;
    }
    if(pos_change_count == 2760){
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[0] =10;//id
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[1] =1;//
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[2] =1;//ch
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_COMMAND= INIT_FIND_POS;
    }
    if(pos_change_count == 2785){
        MCenableFrictionCompensation(0,4,2,DISABLE);
        MCenableFrictionCompensation(1,10,2,DISABLE);
        MCBoardSetSwitchingMode(0,4,0);
        MCBoardSetSwitchingMode(1,10,0);
        MCJointGainOverride(0,4,2,0,500);
        MCJointGainOverride(1,10,2,0,500);
    }

    if(pos_change_count == 2890){
        printf("Debug---------------------------->> Wheel 2 Walk End...!!!\n");
        WB_FLAG = false;
        STATUS_FLAG=STATUS_IDLE;
        WHEELtoWALK_FLAG=false;
        pos_change_count=0;
    }

    // post;
}

void WHEELtoWALK_FUNCTION(void)
{
    // gain back reverse

    if(pos_change_count == 1)
        ;
    ////sharedData->STATE_COMMAND = TCMD_WHEEL_POS_CHANGE_WH2WALK_START;

    if(pos_change_count == 3)
    {
        MCsetFrictionParameter(0,4,2,1000,_GAIN_RWH,0);
        MCsetFrictionParameter(1,10,2,1000,_GAIN_LWH,0);
        MCBoardSetSwitchingMode(0,4,1);
        MCBoardSetSwitchingMode(1,10,1);
        MCenableFrictionCompensation(0,4,2,ENABLE);
        MCenableFrictionCompensation(1,10,2,ENABLE);
        MCJointGainOverride(0,4,2,100,1);
        MCJointGainOverride(1,10,2,100,1);
    }
    if(pos_change_count == 10)
    {
        joint->SetMoveJoint(RHP, -30., 1500, MOVE_ABSOLUTE);//c
        joint->SetMoveJoint(LHP, -30., 1500, MOVE_ABSOLUTE);//c
    }
    // step 3 reverse
    if(pos_change_count == 350)
    {
        joint->SetMoveJoint(RAP, -94.64, 8000, MOVE_ABSOLUTE);//b
        joint->SetMoveJoint(LAP, -94.64, 8000, MOVE_ABSOLUTE);//b
        joint->SetMoveJoint(RKN, 143.9, 8000, MOVE_ABSOLUTE);//b//141
        joint->SetMoveJoint(LKN, 143.9, 8000, MOVE_ABSOLUTE);//b
    }
    if(pos_change_count == 1250)
    {
        joint->SetMoveJoint(RHP, -57.0, 2500, MOVE_ABSOLUTE);//a
        joint->SetMoveJoint(LHP, -57.0, 2500, MOVE_ABSOLUTE);//a
    }
    if(pos_change_count == 1750)
    {
        joint->SetMoveJoint(RHP, -59.32, 1000, MOVE_ABSOLUTE);//b//-55
        joint->SetMoveJoint(LHP, -59.32, 1000, MOVE_ABSOLUTE);//b
    }
    // step 2 reverse
    if(pos_change_count == 1960)
        StartWBIKmotion(-1);
    if(pos_change_count == 1965)
        _FOOT_CENT_X2 =(WBmotion->pRF_3x1[0]+WBmotion->pLF_3x1[0])/2.;
    if(pos_change_count ==1970)
    {
        quat qt_q_pelv=quat(vec3(0,1,0),-45*D2R);
        doubles ds_q_pelv(4);
        for(int k=0;k<4;k++)
            ds_q_pelv[k]=qt_q_pelv[k];
        WBmotion->addPELPosInfo(0.395,2.5);
        WBmotion->addCOMInfo(_FOOT_CENT_X2+0.02, 0., 2.5);
        WBmotion->addPELOriInfo(ds_q_pelv, 2.5);
    }
    // step 1 reverse
    if(pos_change_count == 2500){
        quat qt_q_pelv=quat(vec3(0,1,0),0*D2R);
        doubles ds_q_pelv(4);
        for(int k=0;k<4;k++)
            ds_q_pelv[k]=qt_q_pelv[k];
        WBmotion->addPELPosInfo(_PEL_Z_WALKREADY,3.);
        WBmotion->addCOMInfo(_FOOT_CENT_X2+0.01, 0., 3.);
        WBmotion->addPELOriInfo(ds_q_pelv, 3.);
    }
    if(pos_change_count == 3110){
        WB_FLAG = false;
        sharedCMD->COMMAND[PODO_NO_WALKREADY].USER_PARA_INT[0]=3;
        sharedCMD->COMMAND[PODO_NO_WALKREADY].USER_COMMAND=WALKREADY_GO_WALKREADYPOS;
    }
    if(pos_change_count == 3710){
        //JS
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[0] =4;//id
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[1] =1;//
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[2] =1;//ch
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_COMMAND= INIT_FIND_POS;
         MCJointFindHome(0, MC_GetID(RWH), MC_GetCH(RWH)+1);
    }
    if(pos_change_count == 3730){
        //JS
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[0] =10;//id
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[1] =1;//
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[2] =1;//ch
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_COMMAND= INIT_FIND_POS;
        MCJointFindHome(1, MC_GetID(LWH), MC_GetCH(LWH)+1);
    }
    if(pos_change_count == 3760){
        MCenableFrictionCompensation(0,4,2,DISABLE);
        MCenableFrictionCompensation(1,10,2,DISABLE);
        MCBoardSetSwitchingMode(0,4,0);
        MCBoardSetSwitchingMode(1,10,0);
        MCJointGainOverride(0,4,2,0,500);
        MCJointGainOverride(1,10,2,0,500);
    }
    if(pos_change_count == 3900){
        printf("Debug---------------------------->> Wheel 2 Walk End...!!!\n");
        WB_FLAG = false;
        STATUS_FLAG=STATUS_IDLE;
        WHEELtoWALK_FLAG=false;
        pos_change_count=0;
    }

//    // Go  to walk ready
//    if(pos_change_count == 2500)
//    {
//        WB_FLAG = false;
//        sharedData->USER_PARA_INT[PODO_NO_WALKREADY][0]=3;
//        sharedData->USER_COMMAND[PODO_NO_WALKREADY]=WALKREADY_GO_WALKREADYPOS;
//    }
//    // gain back !!!
//    if(pos_change_count==2800)
//    {
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[0] =4;//id
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[1] =1;//
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[2] =1;//ch
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_COMMAND= INIT_FIND_POS;
//    }
//    if(pos_change_count==2820)
//    {
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[0] =10;//id
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[1] =1;//
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[2] =1;//ch
//        sharedCMD->COMMAND[PODO_NO_DAEMON].USER_COMMAND= INIT_FIND_POS;
//    }
//    if(pos_change_count == 2850)
//    {
//        RBenableFrictionCompensation(0,4,DISABLE,DISABLE);
//        RBenableFrictionCompensation(1,10,DISABLE,DISABLE);
//        RBBoardSetSwitchingMode(0,4,0);
//        RBBoardSetSwitchingMode(1,10,0);
//        RBJointGainOverride(0,4,1000,1000,3000);
//        RBJointGainOverride(1,10,1000,1000,3000);
//    }

//    // terminate
//    if(pos_change_count == 3500)
//    {
//        ////sharedData->STATE_COMMAND = TCMD_WHEEL_POS_CHANGE_WH2WALK_DONE;
//        printf("Debug---------------------------->> Wheel 2 Walk End...!!!\n");
//        WB_FLAG = false;
//        STATUS_FLAG=STATUS_IDLE;
//        WHEELtoWALK_FLAG=false;
//        pos_change_count=0;
//    }
}

//void WheelMoveManual(void)
//{
//    // Stop case

//    if(AROW_RL == 0 && R_JOG_UD == 0){
//        pushData(RWHList, 0.0);
//        pushData(LWHList, 0.0);
//        return;
//    }

//    // Move

//    if(AROW_RL==0)
//    {
//        //Direct
//        double temp_vel = 1.4*((double)R_JOG_UD);//350/200 = 1.75
//        if(L_JOG_RL == 0){
//            pushData(RWHList, temp_vel);
//            pushData(LWHList, temp_vel);
//            return;
//        }
//    }
//    else
//    {
//        // Stady rotation
//        double temp_vel = 0;
//        if(AROW_RL<0){
//            temp_vel = -0.9;//350*0.7/200 = 1.225
//        }else{
//            temp_vel = +0.9;
//        }
//        pushData(RWHList, temp_vel);
//        pushData(LWHList, -temp_vel);
//    }
//}

//
// When we use joystick
//
void WheelMoveManual(void)// when we use joystick
{
    double radius = 0.0;
    double angularV = 0.0;
    const double width_1 = _robot_width;
    double lin_vel = 0.0;

    double vel1, vel2;
    const double wheel_r = _wheel_radi;

    double max_radius=6.0;
    double min_radius=0.4;
    double max_angularV=0.45;

    // Stop case----------------------------------------------------
    if(L_JOG_RL == 0 && R_JOG_UD == 0){
        pushData(RWHList, 0.0);
        pushData(LWHList, 0.0);
        return;
    }
    if(AROW_RL==0)
    {
        //Direct Case----------------------------------------------------

        //lin_vel = ((double)R_JOG_UD)/130000.0;//100:1
        lin_vel = ((double)R_JOG_UD)/65000.0;//50:1
        //lin_vel = ((double)R_JOG_UD)/150000.0;//Optic Test

        if(L_JOG_RL == 0){
            radius = max_radius;//(m)
            angularV = lin_vel/radius;
            vel1 = vel2 = (radius+width_1/2.)*angularV;//(m/s)

            pushData(RWHList, vel1/wheel_r*R2D/200.);
            pushData(LWHList, vel2/wheel_r*R2D/200.);
            return;
        }


        // General Case--------------------------------------------------

        radius = max_radius-(max_radius-min_radius)*fabs((double)L_JOG_RL)/32767.0;//(m)
        angularV = lin_vel/radius;

        if(angularV>max_angularV)
            angularV=max_angularV;
        if(angularV<-max_angularV)
            angularV=-max_angularV;

        if(radius > max_radius) radius = max_radius;
        if(L_JOG_RL < 0){
            vel1 = (radius+width_1/2.)*angularV;//(m/s)
            vel2 = (radius-width_1/2.)*angularV;
        }else{
            vel1 = (radius-width_1/2.)*angularV;//(m/s)
            vel2 = (radius+width_1/2.)*angularV;
        }

        pushData(RWHList, vel1/wheel_r*R2D/200.);
        pushData(LWHList, vel2/wheel_r*R2D/200.);
    }
    else
    {
        // Stady rotation
        float regul_mag = 130000.0;//for normal use
        //float regul_mag = 800000.0;//for BLE
        if(AROW_RL<0){
            vel1 = fabs(((double)R_JOG_UD)/regul_mag);
        }else{
            vel1 = -fabs(((double)R_JOG_UD)/regul_mag);
        }
        vel2 =-vel1;
        pushData(RWHList, vel1/wheel_r*R2D/200.);
        pushData(LWHList, vel2/wheel_r*R2D/200.);
    }
}

void CalculateMovingEverage(void)
{
    double temp1 = 0.0;
    double temp2 = 0.0;
    for(int i=0; i<120; i++){
        temp1 += RWHList[i];
        temp2 += LWHList[i];
    }
    RWHnow = temp1/120.0;
    LWHnow = temp2/120.0;
}
void InitWheelHome(void){
    printf("-------Init Wheel Home RWH / LWH ...!!!-------\n");
    usleep(10*1000);

    sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[0] =0;//id
    sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[1] =4;//id
    sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[2] =1;//ch
    sharedCMD->COMMAND[PODO_NO_DAEMON].USER_COMMAND= DAEMON_INIT_FIND_HOME;

    usleep(400*1000);
    sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[0] =0;//id
    sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[1] =10;//id
    sharedCMD->COMMAND[PODO_NO_DAEMON].USER_PARA_CHAR[2] =1;//ch
    sharedCMD->COMMAND[PODO_NO_DAEMON].USER_COMMAND= DAEMON_INIT_FIND_HOME;
    usleep(400*1000);

}

void Knee_Gain_Over(void){
    MCJointGainOverride(0, 3, 1, 35, 330);
    MCJointGainOverride(1, 9, 1, 35, 330);
    usleep(330*1000);
    MCJointGainOverride(0, 3, 1, 100, 2400);
    MCJointGainOverride(1, 9, 1, 100, 2400);
    usleep(2450*1000);
}

void Knee_Gain_Return(void){
    MCJointGainOverride(0, 3, 1, 25, 900);
    MCJointGainOverride(1, 9, 1, 25, 900);
    usleep(900*1000);
    MCJointGainOverride(0, 3, 1, 0, 2950);
    MCJointGainOverride(1, 9, 1, 0, 2950);
    usleep(3000*1000);
}

void Wheel_Walk_Ready(double pos_ms){
//    joint->SetMoveJoint(RSP, 40.0, pos_ms, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RSR, 10.0, pos_ms, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RSY, 0.0, pos_ms, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(REB, -130, pos_ms, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RWY, 0.0, pos_ms, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RWP, 20.0, pos_ms, MOVE_ABSOLUTE);

//    joint->SetMoveJoint(LSP, 40.0, pos_ms, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LSR, -10.0, pos_ms, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LSY, 0.0, pos_ms, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LEB, -130.0, pos_ms, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LWY, 0.0, pos_ms, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LWP, 20.0, pos_ms, MOVE_ABSOLUTE);

//    joint->SetMoveJoint(RWY2, 0.0, pos_ms, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RF2, 0.0, pos_ms, MOVE_ABSOLUTE);

//    joint->SetMoveJoint(LWY2, 0.0, pos_ms, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LF2, 0.0, pos_ms, MOVE_ABSOLUTE);

    joint->SetMoveJoint(WST, 0.0, pos_ms, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RHY, 0., pos_ms, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHR, 0., pos_ms, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHP, -40.66, pos_ms, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RKN, 75.02, pos_ms, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RAP, -34.35, pos_ms, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RAR, 0., pos_ms, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LHY, 0., pos_ms, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHR, 0., pos_ms, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHP, -40.66, pos_ms, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LKN, 75.02, pos_ms, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LAP, -34.35, pos_ms, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LAR, 0., pos_ms, MOVE_ABSOLUTE);
}
