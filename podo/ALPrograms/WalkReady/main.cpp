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

// This AL is made by Jungwoo Heo (Rainbow Robotics)
// e-mail: jwheo@rainbow.re.kr

#include <QCoreApplication>

#include <iostream>
#include <string>
#include <sys/mman.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>

//hyoin

//#include "../../SHARE/Headers/ik_math2.h"
#include "../../SHARE/Headers/kine_drc_hubo2.h"
#include "../../SHARE/Headers/ik_math4.h"
#include "../../SHARE/Headers/kine_drc_hubo4.h"
#include "ManualCAN.h"
//#include "kine_drc_hubo_tj.h"
#include "joint.h"

#define PODO_AL_NAME       "WALKREADY"

using namespace std;
const int RB_SUCCESS = true;
const int RB_FAIL = false;

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
#define ROW_data_debug2 20000
#define COL_data_debug2 100
double   JW_Data_Debug[COL_data_debug2][ROW_data_debug2];
double   JW_Data_Debug_Filtered[COL_data_debug2][ROW_data_debug2];
int HitMotionCnt = 0;
const static int ReadyStart = 0;
const static int ReadyEnd = 1200;
const static int HitStart = ReadyEnd+1;
const static int HitEnd = 4400;


int cntforcountsleep = 0;
void countsleep(int microsec)
{
    cntforcountsleep = 0;
    while(microsec/5.0/1000.0>cntforcountsleep)
    {
        usleep(1*1000);//1ms sleep
    }
}


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
JointControlClass *joint;
// =====================================================

// Command Set =========================================
enum WALKREADYCOMMAND
{
    WALKREADY_NO_ACT = 100,
    WALKREADY_GO_WALKREADYPOS,
    WALKREADY_GO_HOMEPOS,
    WALKREADY_WHEELCMD,
    WALKREADY_INFINITY,
    WALKREADY_HIT_READY,
    WALKREADY_HIT_HIT,
    WALKREADY_HIT_RETURN,
    WALKREADY_HIT_INIT_POS,
};
// =====================================================

int WB_FLAG = 0;
void DemoReady();
void GotoWalkReadyPos();
void GotoWalkReadyPos3();
void GotoWalkReadyPos4();
void GotoHomePos();
void GotoDrillWidePos();
void GotoWalkReadyPos_with_WST_M180();

void WBIK_PARA_CHANGE();

// Manual CAN for GainOverride
//int	PushCANMessage(MANUAL_CAN MCData);
//int RBJointGainOverride(unsigned int _canch, unsigned int _bno, unsigned int _override1, unsigned int _override2, unsigned int _duration);
//int RBBoardSetSwitchingMode(unsigned int _canch, unsigned int _bno, int _mode);
//int RBenableFrictionCompensation(unsigned int _canch, unsigned int _bno, unsigned int _enable1, unsigned int _enable2);
//int RBindicator(unsigned char number);//0 off n n*100ms on/off -1 on 0x85 0,-1 on off
// hyoin
//CKINE_DRC_HUBO_TJ kine_drc_hubo;
CKINE_DRC_HUBO2 kine_drc_hubo;
//KINE_DRC_HUBO3 kine_drc_hubo3;
KINE_DRC_HUBO4 kine_drc_hubo3;

double WBIK_Q0[34] = {0.,};
double WBIK_Q[34] = {0.,},Qub[34]={0.,};

int infinityTest = false;
int infinityToggle = 0;
int infinityCnt = 0;

void WholebodyInitialize();
void get_zmp2();

double  X_ZMP_Local,Y_ZMP_Local,X_ZMP_Global,Y_ZMP_Global,X_ZMP_REF_Local,Y_ZMP_REF_Local,X_ZMP_REF_Global,Y_ZMP_REF_Global;
double GLOBAL_ZMP_REF_X,GLOBAL_ZMP_REF_Y;
double com1,com2;
double t_thread=0.,t0,t1,t2;
double lamda;
double T10,T21;
double a1,b1,c1;
double a2,b2,c2;
double zc,xc,yc,xRF,yRF,zRF,xLF,yLF,zLF;
double LeftPitch;
double RSR_demo,LSR_demo;
double AddJoint;

double des_pCOM_3x1[3]={0,}, des_qPEL_4x1[4]={1,0,0,0}, des_pRF_3x1[3]={0,}, des_qRF_4x1[4]={1,0,0,0}, des_pLF_3x1[3]={0,}, des_qLF_4x1[4]={1,0,0,0};
int DemoFlag;

double I_ZMP_CON_X=0.f,I_ZMP_CON_Y=0.f;
double I_ZMP_CON_X_last=0.f,I_ZMP_CON_Y_last=0.f;
double Old_I_ZMP_CON_X=0.f,Old_I_ZMP_CON_Y=0.f;
double  LPF_Del_PC_X_DSP_XZMP_CON = 0., LPF_Del_PC_Y_DSP_YZMP_CON = 0.;
double  LPF_Del_PC_X_SSP_XZMP_CON = 0., LPF_Del_PC_Y_SSP_YZMP_CON = 0.;
double  Del_PC_X_DSP_XZMP_CON = 0., Del_PC_Y_DSP_YZMP_CON = 0., Old_Del_PC_X_DSP_XZMP_CON = 0., Old_Del_PC_Y_DSP_YZMP_CON = 0.,
        Del_PC_X_SSP_XZMP_CON = 0., Del_PC_Y_SSP_YZMP_CON = 0., Old_Del_PC_X_SSP_XZMP_CON = 0., Old_Del_PC_Y_SSP_YZMP_CON = 0.;
double  Old_Del_PC_X_DSP_XZMP_CON2 = 0;
double  Old_Del_PC_Y_DSP_YZMP_CON2 = 0;

const double DEL_T = 0.005;
const double    OFFSET_ELB = -20.0;
const double    OFFSET_RSR = -15.0;
const double    OFFSET_LSR = 15.0;
double curRSR,curLSR;
double  final_gain_DSP_ZMP_CON = 0., final_gain_SSP_ZMP_CON = 0.;
unsigned int CNT_final_gain_DSP_ZMP_CON = 0,  CNT_final_gain_SSP_ZMP_CON = 0;
void Kirk_Control();
void Kirk_Control_ssp();
void ZMP_intergral_control();
// DSP ZMP Controller
double  kirkZMPCon_XP1(double u, double ZMP, int zero);
double  kirkZMPCon_YP1(double u, double ZMP, int zero);
double  kirkZMPCon_XP2(double u, double ZMP, int zero);
double  kirkZMPCon_YP2(double u, double ZMP, int zero);

enum FTNAME{
    RAFT = 0,
    LAFT
};
enum DEMOFLAG
{
    FOOTUP=100,
    KNEEDOWN,
    FOOTDOWN,
    BOW,
    HIT_HIT,
    HIT_READY,
    HIT_RETURN
};


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

int main(int argc, char *argv[])
{
    FILE_LOG(logERROR) << "KK";

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

//	// Get PODO No. ---------------------------------------
//	if(argc == 1){
//        FILE_LOG(logERROR) << "No input argument";
//        return 0;
//	}
//	else{
//		QString argStr;
//		argStr.sprintf("%s", argv[1]);
//		PODO_NO = argStr.toInt();
//        cout << endl << endl;
//		cout << "======================================================================" << endl;
//		cout << ">>> Process WalkReady is activated..!!" << endl;
//		cout << ">>> PODO NAME: WALKREADY" << endl;
//		cout << ">>> PODO NO: " << PODO_NO << endl;
//        cout << "======================================================================" << endl;
//	}


	// Initialize RBCore -----------------------------------
    if(RBInitialize() == -1)
        isTerminated = -1;


	// User command cheking --------------------------------
	while(isTerminated == 0)
	{
		usleep(100*1000);

        switch(sharedCMD->COMMAND[PODO_NO].USER_COMMAND)
        {
		case WALKREADY_GO_HOMEPOS:
			cout << ">>> COMMAND: WALKREADY_GO_HOMEPOS" << endl;

			joint->RefreshToCurrentReference();
			joint->SetAllMotionOwner();
			GotoHomePos();
           // RBindicator(10);
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = WALKREADY_NO_ACT;
			break;
		case WALKREADY_GO_WALKREADYPOS:
			cout << ">>> COMMAND: WALKREADY_GO_WALKREADYPOS" << endl;
            usleep(500*1000);
            WBIK_PARA_CHANGE();
            infinityTest= false;
            kine_drc_hubo.C_Torso[1] =0;
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] == 0){
                GotoWalkReadyPos();
                usleep(4000*1000);
                //sharedData->STATE_COMMAND = TCMD_WALKREADY_DONE;
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] == 2){
                GotoDrillWidePos();
                usleep(4000*1000);
                //sharedData->STATE_COMMAND = TCMD_WALKREADY_DONE;
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] == 3){
                GotoWalkReadyPos();
                usleep(4000*1000);
                //sharedData->STATE_COMMAND = TCMD_WALKREADY_DONE;
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] == 7){
                GotoWalkReadyPos3();
                DemoFlag = FOOTDOWN;
                usleep(4000*1000);
//                //sharedData->STATE_COMMAND = TCMD_WHEEL_POS_CHANGE_WH2WALK_DONE;
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] == 8){
                GotoWalkReadyPos4();
                DemoFlag = FOOTDOWN;
                usleep(4000*1000);
//                //sharedData->STATE_COMMAND = TCMD_WHEEL_POS_CHANGE_WH2WALK_DONE;
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] == 10){
                GotoWalkReadyPos_with_WST_M180();
                usleep(5000*1000);
                ////sharedData->STATE_COMMAND = TCMD_WALKREADY_DONE;
            }
            else{
                GotoWalkReadyPos();
                usleep(4000*1000);
                //sharedData->STATE_COMMAND = TCMD_WALKREADY_DONE;
            }
            //RBindicator(1);
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = WALKREADY_NO_ACT;
			break;
        case WALKREADY_WHEELCMD:
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = WALKREADY_NO_ACT;
            break;
        case WALKREADY_INFINITY:
            cout << ">>> COMMAND: WALKREADY_DEMO" << endl;

            if(infinityTest == false)
            {
                WholebodyInitialize();

                joint->RefreshToCurrentReference();
                joint->SetAllMotionOwner();

                int pre_DemoFlag;
                pre_DemoFlag = DemoFlag;
                DemoFlag = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
                if(DemoFlag == FOOTUP)
                {
                    if(pre_DemoFlag == FOOTDOWN)
                    {
                        infinityTest = true;
                        t_thread = 0;
                        double postime = 3000.0;

                        joint->SetMoveJoint(RSP, 0.0, postime, MOVE_ABSOLUTE);//Qub[idRSP] = 0.*D2R;
                        joint->SetMoveJoint(RSR, -20.0, postime, MOVE_ABSOLUTE);//Qub[idRSR] = -40.*D2R;
                        joint->SetMoveJoint(RSY, -60.0, postime, MOVE_ABSOLUTE);//Qub[idRSY] = -30.*D2R;
                        joint->SetMoveJoint(REB, -90, postime, MOVE_ABSOLUTE);//Qub[idREB] = -70.*D2R;
                        joint->SetMoveJoint(RWY, 0.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(RWP, 60.0, postime, MOVE_ABSOLUTE);

                        joint->SetMoveJoint(LSP, 0.0, postime, MOVE_ABSOLUTE);//Qub[idLSP] = 0.*D2R;
                        joint->SetMoveJoint(LSR, 20.0, postime, MOVE_ABSOLUTE);//Qub[idLSR] = 40.*D2R;
                        joint->SetMoveJoint(LSY, 60.0, postime, MOVE_ABSOLUTE);//Qub[idLSY] = 30.*D2R;
                        joint->SetMoveJoint(LEB, -90.0, postime, MOVE_ABSOLUTE);//Qub[idLEB] = -70.*D2R;
                        joint->SetMoveJoint(LWY, 0.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LWP, 60.0, postime, MOVE_ABSOLUTE);

//                        Qub[idRWP] = 20.*D2R;
//                        Qub[idLWP] = 20.*D2R;
                        joint->SetMoveJoint(WST, 0.0, postime, MOVE_ABSOLUTE);

                        joint->SetMoveJoint(RWY2, 0.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(RHAND, 0.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LWY2, 0.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LHAND, 0.0, postime, MOVE_ABSOLUTE);
                    }
                    else
                    {
                        DemoFlag = pre_DemoFlag;
                    }
                }
                else if(DemoFlag == FOOTDOWN)
                {
                    if(pre_DemoFlag == FOOTUP || pre_DemoFlag == KNEEDOWN)
                    {
                        infinityTest = true;
                        t_thread = 0;
                        double postime = 4000.0;

                        joint->SetMoveJoint(RSP, 40.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(RSR, 10.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(RSY, 0.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(REB, -130, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(RWY, 0.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(RWP, 20.0, postime, MOVE_ABSOLUTE);

                        joint->SetMoveJoint(LSP, 40.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LSR, -10.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LEB, -130.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LWY, 0.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LWP, 20.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(WST, 0.0, postime, MOVE_ABSOLUTE);

                        joint->SetMoveJoint(RWY2, 0.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(RHAND, 0.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LWY2, 0.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LHAND, 0.0, postime, MOVE_ABSOLUTE);
                    }
                    else
                    {
                        DemoFlag = pre_DemoFlag;
                    }
                }
                else if(DemoFlag == KNEEDOWN)
                {
                    if(pre_DemoFlag == FOOTUP || pre_DemoFlag == KNEEDOWN)
                    {
                        curRSR = joint->GetJointRefAngle(RSR);
                        curLSR = joint->GetJointRefAngle(LSR);
                        infinityTest = true;
                        t_thread = 0;
                    }
                    else
                    {
                        DemoFlag = pre_DemoFlag;
                    }
                }
                else if(DemoFlag == BOW)
                {

                    infinityTest = true;
                    t_thread = 0;
                    double postime = 1300.0;
//                    joint->SetMoveJoint(RSP, 20.0, postime, MOVE_ABSOLUTE);
//                    joint->SetMoveJoint(RSR, 0.0, postime, MOVE_ABSOLUTE);
//                    joint->SetMoveJoint(RSY, 0.0, postime, MOVE_ABSOLUTE);
//                    joint->SetMoveJoint(REB, -50, postime, MOVE_ABSOLUTE);
//                    joint->SetMoveJoint(RWY, 0.0, postime, MOVE_ABSOLUTE);
//                    joint->SetMoveJoint(RWP, 20.0, postime, MOVE_ABSOLUTE);

//                    joint->SetMoveJoint(LSP, 20.0, postime, MOVE_ABSOLUTE);
//                    joint->SetMoveJoint(LSR, 0.0, postime, MOVE_ABSOLUTE);
//                    joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
//                    joint->SetMoveJoint(LEB, -50.0, postime, MOVE_ABSOLUTE);
//                    joint->SetMoveJoint(LWY, 0.0, postime, MOVE_ABSOLUTE);
//                    joint->SetMoveJoint(LWP, 20.0, postime, MOVE_ABSOLUTE);
//                    usleep(1600*1000);
//                    joint->SetMoveJoint(RSP, 40.0, postime, MOVE_ABSOLUTE);
//                    joint->SetMoveJoint(RSR, 10.0, postime, MOVE_ABSOLUTE);
//                    joint->SetMoveJoint(RSY, 0.0, postime, MOVE_ABSOLUTE);
//                    joint->SetMoveJoint(REB, -130, postime, MOVE_ABSOLUTE);
//                    joint->SetMoveJoint(RWY, 0.0, postime, MOVE_ABSOLUTE);
//                    joint->SetMoveJoint(RWP, 20.0, postime, MOVE_ABSOLUTE);

//                    joint->SetMoveJoint(LSP, 40.0, postime, MOVE_ABSOLUTE);
//                    joint->SetMoveJoint(LSR, -10.0, postime, MOVE_ABSOLUTE);
//                    joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
//                    joint->SetMoveJoint(LEB, -130.0, postime, MOVE_ABSOLUTE);
//                    joint->SetMoveJoint(LWY, 0.0, postime, MOVE_ABSOLUTE);
//                    joint->SetMoveJoint(LWP, 20.0, postime, MOVE_ABSOLUTE);
                }
                else if(DemoFlag == HIT_HIT)
                {
                    infinityTest = true;
                    t_thread = 0;
                    joint->RefreshToCurrentReference();
                    joint->SetAllMotionOwner();

                    HitMotionCnt = HitStart;

                    WB_FLAG = -2;
                    countsleep(6000*1000);
                    joint->SetMoveJoint(WST,60,1500,MOVE_ABSOLUTE);
                    countsleep(5000*1000);
                    joint->SetMoveJoint(WST,0,3500,MOVE_ABSOLUTE);
                    countsleep(8000*1000);
                    infinityTest = false;
                    countsleep(100*1000);
                    printf("hit done...\n");                    
                }
                else if(DemoFlag == HIT_READY)
                {
                    DemoReady();
                    countsleep(300*1000);
                    infinityTest = true;
                    t_thread = 0;
                    joint->SetJointRefAngle(RHAND, -125);//open
                    joint->SetMoveJoint(RSP, JW_Data_Debug[69][ReadyStart],5000,MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RSR, JW_Data_Debug[70][ReadyStart]-30,5000,MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RSY, JW_Data_Debug[71][ReadyStart],5000,MOVE_ABSOLUTE);
                    joint->SetMoveJoint(REB, JW_Data_Debug[72][ReadyStart]-15,5000,MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RWY, JW_Data_Debug[73][ReadyStart],5000,MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RWP, JW_Data_Debug[74][ReadyStart],5000,MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RWY2, JW_Data_Debug[75][ReadyStart],5000,MOVE_ABSOLUTE);
                    countsleep(1600*1000);
                    joint->SetMoveJoint(LSP, JW_Data_Debug[76][ReadyStart],5000,MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LSR, JW_Data_Debug[77][ReadyStart],5000,MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LSY, JW_Data_Debug[78][ReadyStart],5000,MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LEB, JW_Data_Debug[79][ReadyStart],5000,MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LWY, JW_Data_Debug[80][ReadyStart],5000,MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LWP, JW_Data_Debug[81][ReadyStart],5000,MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LWY2, JW_Data_Debug[82][ReadyStart],5000,MOVE_ABSOLUTE);
                    joint->SetMoveJoint(WST, -30,5000,MOVE_ABSOLUTE);

                    countsleep(5100*1000);
                    joint->SetJointRefAngle(RHAND, 0);//stop
                    joint->SetMoveJoint(RSR, JW_Data_Debug[70][ReadyStart],3000,MOVE_ABSOLUTE);
                    joint->SetMoveJoint(REB, JW_Data_Debug[72][ReadyStart],3000,MOVE_ABSOLUTE);
                    countsleep(3100*1000);
                    joint->SetJointRefAngle(RHAND, 125);//grab
                    countsleep(3100*1000);
                    //gain override on here
                    short temp_time = 1000;
                    MCJointGainOverride(2, 16, 1, 65, temp_time);//RWY
                    MCJointGainOverride(2, 16, 2, 40, temp_time);//RWP
                    MCJointGainOverride(2, 36, 1, 40, temp_time);//RWY2
                    MCJointGainOverride(3, 37, 1, 40, temp_time);//LWY2
                    countsleep((temp_time+50)*1000);
                    //gain override on here

                    HitMotionCnt = ReadyStart;
                    WB_FLAG = -1;
                    countsleep(6000*1000);
                    infinityTest = false;
                    countsleep(100*1000);
                    printf("hit ready...\n");
                }
                else if(DemoFlag == HIT_RETURN)
                {
                    infinityTest = true;
                    t_thread = 0;
                    joint->SetJointRefAngle(RHAND, -125);//open
                    countsleep(5000*1000);
                    joint->SetMoveJoint(RSR, joint->GetJointRefAngle(RSR)-30,3000,MOVE_ABSOLUTE);

                    joint->SetMoveJoint(REB, joint->GetJointRefAngle(REB)-20,3000,MOVE_ABSOLUTE);
                    countsleep(2100*1000);
                    //gain override off here
                    short temp_time = 1000;
                    MCJointGainOverride(2, 16, 1, 0, temp_time);//RWY
                    MCJointGainOverride(2, 16, 2, 0, temp_time);//RWP
                    MCJointGainOverride(2, 36, 1, 0, temp_time);//RWY2

                    MCJointGainOverride(3, 37, 1, 0, temp_time);//LWY2
                    countsleep((temp_time+50)*1000);

                    double postime = 4000.0;

                    joint->SetMoveJoint(LSP, 40.0, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LSR, -10.0, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LEB, -130.0, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LWY, 0.0, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LWP, 20.0, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LWY2, 90.0, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LHAND, 0.0, postime, MOVE_ABSOLUTE);

                    countsleep(2500*1000);
                    joint->SetJointRefAngle(RHAND, 125);//close
                    joint->SetMoveJoint(RSP, 40.0, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RSR, 10.0, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RSY, 0.0, postime-1500, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(REB, -130, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RWY, 0.0, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RWP, 20.0, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RWY2, 0.0, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RHAND, 0.0, postime, MOVE_ABSOLUTE);
                //    joint->SetMoveJoint(RF3, 0.0, postime, MOVE_ABSOLUTE);

                //
                    countsleep(4100*1000);
                    infinityTest = false;
                    countsleep(100*1000);
                    //gain override off here
                    printf("hit return done...\n");

                }

            }

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = WALKREADY_NO_ACT;
            break;
        case WALKREADY_HIT_READY:
        {
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();
            joint->SetJointRefAngle(RHAND, -125);//open
            joint->SetMoveJoint(RSP, JW_Data_Debug[69][ReadyStart],5000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(RSR, JW_Data_Debug[70][ReadyStart]-30,5000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(RSY, JW_Data_Debug[71][ReadyStart],5000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(REB, JW_Data_Debug[72][ReadyStart]-15,5000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(RWY, JW_Data_Debug[73][ReadyStart],5000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(RWP, JW_Data_Debug[74][ReadyStart],5000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(RWY2, JW_Data_Debug[75][ReadyStart],5000,MOVE_ABSOLUTE);
            countsleep(1600*1000);
            joint->SetMoveJoint(LSP, JW_Data_Debug[76][ReadyStart],5000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(LSR, JW_Data_Debug[77][ReadyStart],5000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(LSY, JW_Data_Debug[78][ReadyStart],5000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(LEB, JW_Data_Debug[79][ReadyStart],5000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(LWY, JW_Data_Debug[80][ReadyStart],5000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(LWP, JW_Data_Debug[81][ReadyStart],5000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(LWY2, JW_Data_Debug[82][ReadyStart],5000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(WST, -30,5000,MOVE_ABSOLUTE);

            //////////COM_BACK
            double des_pCOM_3x1[3], des_pPCz, des_qPEL_4x1[4], des_pRF_3x1[3], des_qRF_4x1[4], des_pLF_3x1[3], des_qLF_4x1[4];

            WBIK_PARA_CHANGE();

            des_pCOM_3x1[0] = 0.0-0.015;//0.0237f;
            des_pCOM_3x1[1] = 0.0;
            des_pCOM_3x1[2] = 0.77;//0.77;// + 0.11;//(61.8kg:0.74)//59//0.8;//71kg

            des_qPEL_4x1[0] = 1.;
            des_qPEL_4x1[1] = 0.;
            des_qPEL_4x1[2] = 0.;
            des_qPEL_4x1[3] = 0.;

            des_pRF_3x1[0] = 0.;
            des_pRF_3x1[1] = -kine_drc_hubo.L_PEL2PEL/2.;//-0.13;//-kine_drc_hubo.L_PEL2PEL/2;//-0.135;//
            des_pRF_3x1[2] = 0.;

            des_qRF_4x1[0] = 1.;
            des_qRF_4x1[1] = 0.;
            des_qRF_4x1[2] = 0.;
            des_qRF_4x1[3] = 0.;

            des_pLF_3x1[0] = 0.;
            des_pLF_3x1[1] = kine_drc_hubo.L_PEL2PEL/2.;//0.13;//kine_drc_hubo.L_PEL2PEL/2;//0.135;//
            des_pLF_3x1[2] = 0.;

            des_qLF_4x1[0] = 1.;
            des_qLF_4x1[1] = 0.;
            des_qLF_4x1[2] = 0.;
            des_qLF_4x1[3] = 0.;

            Qub[idRSP] = 40.*D2R;
            Qub[idLSP] = 40.*D2R;

            Qub[idRSR] = 10.*D2R;
            Qub[idLSR] = -10.*D2R;

            Qub[idRSY] = 0.*D2R;
            Qub[idLSY] = 0.*D2R;

            Qub[idREB] = -130.*D2R;
            Qub[idLEB] = -130.*D2R;

            Qub[idRWY] = 0.*D2R;
            Qub[idLWY] = 0.*D2R;

            Qub[idRWP] = 20.*D2R;
            Qub[idLWP] = 20.*D2R;

            WBIK_Q0[idRHY] = 0.;
            WBIK_Q0[idRHR] = -2.78*D2R;
            WBIK_Q0[idRHP] = -43.9*D2R;
            WBIK_Q0[idRKN] = 80.58*D2R; //77.4*D2R;
            WBIK_Q0[idRAP] = -36.68*D2R;
            WBIK_Q0[idRAR] = 2.78*D2R;

            WBIK_Q0[idLHY] = 0.;
            WBIK_Q0[idLHR] = 2.78*D2R;
            WBIK_Q0[idLHP] = -43.9*D2R;
            WBIK_Q0[idLKN] = 80.58*D2R;//77.4*D2R;
            WBIK_Q0[idLAP] = -36.68*D2R;
            WBIK_Q0[idLAR] = -2.78*D2R;
            WBIK_Q0[idRSP] = 40.*D2R;
            WBIK_Q0[idLSP] = 40.*D2R;

            WBIK_Q0[idRSR] = 10.*D2R;
            WBIK_Q0[idLSR] = -10.*D2R;

            WBIK_Q0[idRSY] = 0.*D2R;
            WBIK_Q0[idLSY] = 0.*D2R;

            WBIK_Q0[idREB] = -130.*D2R;
            WBIK_Q0[idLEB] = -130.*D2R;

            WBIK_Q0[idRWY] = 0.*D2R;
            WBIK_Q0[idLWY] = 0.*D2R;

            WBIK_Q0[idRWP] = 20.*D2R;
            WBIK_Q0[idLWP] = 20.*D2R;
            WBIK_Q0[idRWY2] = 0;//20.*D2R;
            WBIK_Q0[idLWY2] = 0;//20.*D2R;
            WBIK_Q0[idWST] = 0;//-180*D2R;//20.*D2R;
        //    (const double Q_ub_34x1[], const double des_pCOM_3x1[], const double des_qPEL_4x1[],
        //                                const double des_pRF_3x1[], const double des_qRF_4x1[],
        //                                const double des_pLF_3x1[], const double des_qLF_4x1[],
        //                                double Q_return_34x1[]);
            kine_drc_hubo3.IK_LowerBody_Global(WBIK_Q0,Qub,des_pCOM_3x1, des_qPEL_4x1, des_pRF_3x1, des_qRF_4x1, des_pLF_3x1, des_qLF_4x1,WBIK_Q);

            joint->SetMoveJoint(RHY, WBIK_Q[idRHY]*R2D, 5000, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RHR, WBIK_Q[idRHR]*R2D, 5000, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RHP, WBIK_Q[idRHP]*R2D, 5000, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RKN, WBIK_Q[idRKN]*R2D, 5000, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RAP, WBIK_Q[idRAP]*R2D, 5000, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RAR, WBIK_Q[idRAR]*R2D, 5000, MOVE_ABSOLUTE);

            joint->SetMoveJoint(LHY, WBIK_Q[idLHY]*R2D, 5000, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LHR, WBIK_Q[idLHR]*R2D, 5000, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LHP, WBIK_Q[idLHP]*R2D, 5000, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LKN, WBIK_Q[idLKN]*R2D, 5000, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LAP, WBIK_Q[idLAP]*R2D, 5000, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LAR, WBIK_Q[idLAR]*R2D, 5000, MOVE_ABSOLUTE);


            //////////COM_BACK

            countsleep(5100*1000);
            joint->SetJointRefAngle(RHAND, 0);//stop
            joint->SetMoveJoint(RSR, JW_Data_Debug[70][ReadyStart],3000,MOVE_ABSOLUTE);
            joint->SetMoveJoint(REB, JW_Data_Debug[72][ReadyStart],3000,MOVE_ABSOLUTE);
            countsleep(3100*1000);
            joint->SetJointRefAngle(RHAND, 125);//grab
            countsleep(3100*1000);
            //gain override on here
            short temp_time = 1000;
//            MCJointGainOverride(2, 13, 1, 40, temp_time);//RSP
//            MCJointGainOverride(2, 14, 1, 40, temp_time);//RSR
//            MCJointGainOverride(2, 15, 1, 40, temp_time);//RSY
//            MCJointGainOverride(2, 15, 2, 40, temp_time);//REB
            MCJointGainOverride(2, 16, 1, 65, temp_time);//RWY
            MCJointGainOverride(2, 16, 2, 40, temp_time);//RWP
            MCJointGainOverride(2, 36, 1, 40, temp_time);//RWY2

//            MCJointGainOverride(3, 19, 1, 40, temp_time);//LSY
//            MCJointGainOverride(3, 19, 2, 40, temp_time);//LEB
//            MCJointGainOverride(3, 20, 1, 65, temp_time);//LWY
//            MCJointGainOverride(3, 20, 2, 40, temp_time);//LWP
            MCJointGainOverride(3, 37, 1, 40, temp_time);//LWY2
            countsleep((temp_time+50)*1000);
            //gain override on here

            HitMotionCnt = ReadyStart;
            WB_FLAG = -1;
            countsleep(6000*1000);
            printf("hit ready...\n");
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = WALKREADY_NO_ACT;
        }
        break;
        case WALKREADY_HIT_HIT:
        {
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();


            HitMotionCnt = HitStart;

            WB_FLAG = -2;
            countsleep(5000*1000);
            joint->SetMoveJoint(WST,10,4500,MOVE_ABSOLUTE);
            countsleep(6000*1000);
            joint->SetMoveJoint(WST,0,3500,MOVE_ABSOLUTE);
            countsleep(8000*1000);
            printf("hit done...\n");
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = WALKREADY_NO_ACT;
        }
        break;
        case WALKREADY_HIT_RETURN:
        {
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();
            joint->SetJointRefAngle(RHAND, -125);//open
            countsleep(5000*1000);
            joint->SetMoveJoint(RSR, joint->GetJointRefAngle(RSR)-30,3000,MOVE_ABSOLUTE);

            joint->SetMoveJoint(REB, joint->GetJointRefAngle(REB)-20,3000,MOVE_ABSOLUTE);
            countsleep(2100*1000);
            //gain override off here
            short temp_time = 1000;
//            MCJointGainOverride(2, 13, 1, 0, temp_time);//RSP
//            MCJointGainOverride(2, 14, 1, 0, temp_time);//RSR
//            MCJointGainOverride(2, 15, 1, 0, temp_time);//RSY
//            MCJointGainOverride(2, 15, 2, 0, temp_time);//REB
            MCJointGainOverride(2, 16, 1, 0, temp_time);//RWY
            MCJointGainOverride(2, 16, 2, 0, temp_time);//RWP
            MCJointGainOverride(2, 36, 1, 0, temp_time);//RWY2

//            MCJointGainOverride(3, 19, 1, 0, temp_time);//LSY
//            MCJointGainOverride(3, 19, 2, 0, temp_time);//LEB
//            MCJointGainOverride(3, 20, 1, 0, temp_time);//LWY
            //MCJointGainOverride(3, 20, 2, 0, temp_time);//LWP
            MCJointGainOverride(3, 37, 1, 0, temp_time);//LWY2
            countsleep((temp_time+50)*1000);

            /////COM RETURN
            double des_pCOM_3x1[3], des_pPCz, des_qPEL_4x1[4], des_pRF_3x1[3], des_qRF_4x1[4], des_pLF_3x1[3], des_qLF_4x1[4];

            WBIK_PARA_CHANGE();

            des_pCOM_3x1[0] = 0.0;//0.0237f;
            des_pCOM_3x1[1] = 0.0;
            des_pCOM_3x1[2] = 0.77;//0.77;// + 0.11;//(61.8kg:0.74)//59//0.8;//71kg

            des_qPEL_4x1[0] = 1.;
            des_qPEL_4x1[1] = 0.;
            des_qPEL_4x1[2] = 0.;
            des_qPEL_4x1[3] = 0.;

            des_pRF_3x1[0] = 0.;
            des_pRF_3x1[1] = -kine_drc_hubo.L_PEL2PEL/2.;//-0.13;//-kine_drc_hubo.L_PEL2PEL/2;//-0.135;//
            des_pRF_3x1[2] = 0.;

            des_qRF_4x1[0] = 1.;
            des_qRF_4x1[1] = 0.;
            des_qRF_4x1[2] = 0.;
            des_qRF_4x1[3] = 0.;

            des_pLF_3x1[0] = 0.;
            des_pLF_3x1[1] = kine_drc_hubo.L_PEL2PEL/2.;//0.13;//kine_drc_hubo.L_PEL2PEL/2;//0.135;//
            des_pLF_3x1[2] = 0.;

            des_qLF_4x1[0] = 1.;
            des_qLF_4x1[1] = 0.;
            des_qLF_4x1[2] = 0.;
            des_qLF_4x1[3] = 0.;

            Qub[idRSP] = 40.*D2R;
            Qub[idLSP] = 40.*D2R;

            Qub[idRSR] = 10.*D2R;
            Qub[idLSR] = -10.*D2R;

            Qub[idRSY] = 0.*D2R;
            Qub[idLSY] = 0.*D2R;

            Qub[idREB] = -130.*D2R;
            Qub[idLEB] = -130.*D2R;

            Qub[idRWY] = 0.*D2R;
            Qub[idLWY] = 0.*D2R;

            Qub[idRWP] = 20.*D2R;
            Qub[idLWP] = 20.*D2R;

            WBIK_Q0[idRHY] = 0.;
            WBIK_Q0[idRHR] = -2.78*D2R;
            WBIK_Q0[idRHP] = -43.9*D2R;
            WBIK_Q0[idRKN] = 80.58*D2R; //77.4*D2R;
            WBIK_Q0[idRAP] = -36.68*D2R;
            WBIK_Q0[idRAR] = 2.78*D2R;

            WBIK_Q0[idLHY] = 0.;
            WBIK_Q0[idLHR] = 2.78*D2R;
            WBIK_Q0[idLHP] = -43.9*D2R;
            WBIK_Q0[idLKN] = 80.58*D2R;//77.4*D2R;
            WBIK_Q0[idLAP] = -36.68*D2R;
            WBIK_Q0[idLAR] = -2.78*D2R;
            WBIK_Q0[idRSP] = 40.*D2R;
            WBIK_Q0[idLSP] = 40.*D2R;

            WBIK_Q0[idRSR] = 10.*D2R;
            WBIK_Q0[idLSR] = -10.*D2R;

            WBIK_Q0[idRSY] = 0.*D2R;
            WBIK_Q0[idLSY] = 0.*D2R;

            WBIK_Q0[idREB] = -130.*D2R;
            WBIK_Q0[idLEB] = -130.*D2R;

            WBIK_Q0[idRWY] = 0.*D2R;
            WBIK_Q0[idLWY] = 0.*D2R;

            WBIK_Q0[idRWP] = 20.*D2R;
            WBIK_Q0[idLWP] = 20.*D2R;
            WBIK_Q0[idRWY2] = 0;//20.*D2R;
            WBIK_Q0[idLWY2] = 0;//20.*D2R;
            WBIK_Q0[idWST] = 0;//-180*D2R;//20.*D2R;
        //    (const double Q_ub_34x1[], const double des_pCOM_3x1[], const double des_qPEL_4x1[],
        //                                const double des_pRF_3x1[], const double des_qRF_4x1[],
        //                                const double des_pLF_3x1[], const double des_qLF_4x1[],
        //                                double Q_return_34x1[]);
            kine_drc_hubo3.IK_LowerBody_Global(WBIK_Q0,Qub,des_pCOM_3x1, des_qPEL_4x1, des_pRF_3x1, des_qRF_4x1, des_pLF_3x1, des_qLF_4x1,WBIK_Q);

            joint->SetMoveJoint(RHY, WBIK_Q[idRHY]*R2D, 5000, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RHR, WBIK_Q[idRHR]*R2D, 5000, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RHP, WBIK_Q[idRHP]*R2D, 5000, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RKN, WBIK_Q[idRKN]*R2D, 5000, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RAP, WBIK_Q[idRAP]*R2D, 5000, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RAR, WBIK_Q[idRAR]*R2D, 5000, MOVE_ABSOLUTE);

            joint->SetMoveJoint(LHY, WBIK_Q[idLHY]*R2D, 5000, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LHR, WBIK_Q[idLHR]*R2D, 5000, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LHP, WBIK_Q[idLHP]*R2D, 5000, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LKN, WBIK_Q[idLKN]*R2D, 5000, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LAP, WBIK_Q[idLAP]*R2D, 5000, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LAR, WBIK_Q[idLAR]*R2D, 5000, MOVE_ABSOLUTE);

            /// COM RETURN
            double postime = 4000.0;



            joint->SetMoveJoint(LSP, 40.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LSR, -10.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LEB, -130.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LWY, 0.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LWP, 20.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LWY2, 90.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LHAND, 0.0, postime, MOVE_ABSOLUTE);

            countsleep(2500*1000);
            joint->SetJointRefAngle(RHAND, 125);//close
            joint->SetMoveJoint(RSP, 40.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RSR, 10.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RSY, 0.0, postime-1500, MOVE_ABSOLUTE);
            joint->SetMoveJoint(REB, -130, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RWY, 0.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RWP, 20.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RWY2, 0.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RHAND, 0.0, postime, MOVE_ABSOLUTE);
        //    joint->SetMoveJoint(RF3, 0.0, postime, MOVE_ABSOLUTE);

        //
            countsleep(4100*1000);
            //gain override off here
            printf("hit return done...\n");
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = WALKREADY_NO_ACT;
        }
        break;
        case WALKREADY_HIT_INIT_POS:
        {
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();
            double postime = 4000.0;



            joint->SetMoveJoint(LSP, 40.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LSR, -10.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LEB, -130.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LWY, 0.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LWP, 20.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LWY2, 90.0, postime, MOVE_ABSOLUTE);

            joint->SetJointRefAngle(LHAND, -125);//open
            joint->SetMoveJoint(RSP, 40.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RSR, 10.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RSY, 0.0, postime-1500, MOVE_ABSOLUTE);
            joint->SetMoveJoint(REB, -130, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RWY, 0.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RWP, 20.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RWY2, 0.0, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RHAND, 0.0, postime, MOVE_ABSOLUTE);


            countsleep(4100*1000);
            joint->SetJointRefAngle(LHAND, 0);//stop
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = WALKREADY_NO_ACT;
        }
        break;

		default:
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = WALKREADY_NO_ACT;
			break;
		}
	}
    FILE_LOG(logERROR) << "Process WalkReady is terminated" << endl;
	return 0;
}
// --------------------------------------------------------------------------------------------- //



// --------------------------------------------------------------------------------------------- //
void *RBTaskThread(void *)
{
	while(isTerminated == 0)
    {
        checkSuspend();


        if(WB_FLAG ==-1)//HIT_READY
        {
            joint->SetJointRefAngle(RSP, JW_Data_Debug[69][HitMotionCnt]);
            joint->SetJointRefAngle(RSR, JW_Data_Debug[70][HitMotionCnt]);
            joint->SetJointRefAngle(RSY, JW_Data_Debug[71][HitMotionCnt]);
            joint->SetJointRefAngle(REB, JW_Data_Debug[72][HitMotionCnt]);
            joint->SetJointRefAngle(RWY, JW_Data_Debug[73][HitMotionCnt]);
            joint->SetJointRefAngle(RWP, JW_Data_Debug[74][HitMotionCnt]);
            joint->SetJointRefAngle(RWY2, JW_Data_Debug[75][HitMotionCnt]);

            joint->SetJointRefAngle(LSP, JW_Data_Debug[76][HitMotionCnt]);
            joint->SetJointRefAngle(LSR, JW_Data_Debug[77][HitMotionCnt]);
            joint->SetJointRefAngle(LSY, JW_Data_Debug[78][HitMotionCnt]);
            joint->SetJointRefAngle(LEB, JW_Data_Debug[79][HitMotionCnt]);
            joint->SetJointRefAngle(LWY, JW_Data_Debug[80][HitMotionCnt]);
            joint->SetJointRefAngle(LWP, JW_Data_Debug[81][HitMotionCnt]);
            joint->SetJointRefAngle(LWY2, JW_Data_Debug[82][HitMotionCnt]);

            HitMotionCnt++;
            if(HitMotionCnt>ReadyEnd)
            {
                WB_FLAG = 0;
            }
        }
        if(WB_FLAG ==-2)//HIT_HIT
        {
            joint->SetJointRefAngle(RSP, JW_Data_Debug[69][HitMotionCnt]);
            joint->SetJointRefAngle(RSR, JW_Data_Debug[70][HitMotionCnt]);
            joint->SetJointRefAngle(RSY, JW_Data_Debug[71][HitMotionCnt]);
            joint->SetJointRefAngle(REB, JW_Data_Debug[72][HitMotionCnt]);
            joint->SetJointRefAngle(RWY, JW_Data_Debug[73][HitMotionCnt]);
            joint->SetJointRefAngle(RWP, JW_Data_Debug[74][HitMotionCnt]);
            joint->SetJointRefAngle(RWY2, JW_Data_Debug[75][HitMotionCnt]);

            joint->SetJointRefAngle(LSP, JW_Data_Debug[76][HitMotionCnt]);
            joint->SetJointRefAngle(LSR, JW_Data_Debug[77][HitMotionCnt]);
            joint->SetJointRefAngle(LSY, JW_Data_Debug[78][HitMotionCnt]);
            joint->SetJointRefAngle(LEB, JW_Data_Debug[79][HitMotionCnt]);
            joint->SetJointRefAngle(LWY, JW_Data_Debug[80][HitMotionCnt]);
            joint->SetJointRefAngle(LWP, JW_Data_Debug[81][HitMotionCnt]);
            joint->SetJointRefAngle(LWY2, JW_Data_Debug[82][HitMotionCnt]);

            HitMotionCnt++;
            if(HitMotionCnt>HitEnd)
            {
                WB_FLAG = 0;
            }
        }

        if(infinityTest == true)
        {
            double temp1des_qLF_4x1[4],temp2des_qLF_4x1[4],temp3des_qLF_4x1[4],temp4des_qLF_4x1[4],temp5des_qLF_4x1[4];
            double temp1des_qPEL_4x1[4],temp2des_qPEL_4x1[4],temp3des_qPEL_4x1[4],temp4des_qPEL_4x1[4],temp5des_qPEL_4x1[4];
            double  sagging = 1.;
            double  com_add = 0.001;
            double PELx;
            get_zmp2();

            if(DemoFlag == FOOTUP)
            {
                com1=0;
                com2=-kine_drc_hubo.L_PEL2PEL/2. - com_add ;
                t1=2.5;
                t2=t1*2.;
            }
            else if(DemoFlag == KNEEDOWN)
            {
                com1=-kine_drc_hubo.L_PEL2PEL/2. - com_add;
                com2=-kine_drc_hubo.L_PEL2PEL/2. - com_add;
                t1=1.8;
                t2=t1*2.;
            }
            else if(DemoFlag == FOOTDOWN)
            {
                com1=-kine_drc_hubo.L_PEL2PEL/2. - com_add;
                com2=0;
                t1=2.;
                t2=t1*2.;
            }
            else if(DemoFlag == BOW)
            {
                t1=1.5;
                t2=t1*2.;
            }
            else if(DemoFlag == HIT_HIT)
            {
                t1=9.5;
                t2=t1*2.;
            }
            else if(DemoFlag == HIT_READY)
            {
                t1=9.5;
                t2=t1*2.;
            }
            else if(DemoFlag == HIT_RETURN)
            {
                t1=6.;
                t2=t1*2.;
            }

            t0=0;

            lamda = sqrt(9.81/0.85);

            T10=exp(lamda*(t1-t0));
            T21=exp(lamda*(t2-t1));


            c1 = (-T10*(com1 - com2)*(T21 +1))/(2*(T10*T21-1)*(T10-1));
            b1 = c1;
            a1 = com1 - 2*c1;

            c2 = ((com1 - com2)*(T10 +1))/(2*(T10*T21-1)*(T21-1));
            b2 = T21*T21*c2;
            a2 = com2 - 2*T21*c2;

            //==============  COM & Ref ZMP Trajectory ================//
            if(t_thread>=t0 && t_thread<t1)
            {
                GLOBAL_ZMP_REF_X =0;
                GLOBAL_ZMP_REF_Y =a1;

//                        yc = a1 + b1*exp(-lamda*(t_thread-t0))+c1*exp(lamda*(t_thread-t0));
                if(DemoFlag == FOOTUP)
                    yc = (-kine_drc_hubo.L_PEL2PEL/2. - com_add)*0.5*(1-cos(PI*(t_thread)/(t2-t0-1.5)));
                else if(DemoFlag == FOOTDOWN)
                    yc = a1 + b1*exp(-lamda*(t_thread-t0))+c1*exp(lamda*(t_thread-t0));
            }
            else if(t_thread>=t1 && t_thread<(t2))
            {
                GLOBAL_ZMP_REF_X =0;
                GLOBAL_ZMP_REF_Y =a2;
//                        yc = a2 + b2*exp(-lamda*(t_thread-t1))+c2*exp(lamda*(t_thread-t1));
                if(DemoFlag == FOOTUP)
                {
                    if(t_thread < (t2-1.5))
                    yc = (-kine_drc_hubo.L_PEL2PEL/2. - com_add)*0.5*(1-cos(PI*(t_thread)/(t2-t0-1.5)));
                }
                else if(DemoFlag == FOOTDOWN)
                    yc = a2 + b2*exp(-lamda*(t_thread-t1))+c2*exp(lamda*(t_thread-t1));
            }

            if(DemoFlag == BOW || DemoFlag == HIT_HIT|| DemoFlag == HIT_RETURN|| DemoFlag == HIT_READY)
            {
                GLOBAL_ZMP_REF_X =0.;
                GLOBAL_ZMP_REF_Y =0.;
                yc =0;
            }
            //========================================================//

            //==============  qPEL Trajectory ================//

            if(DemoFlag == BOW)
            {
                if(t_thread>=t0 && t_thread<t2)
                {
                    PELx = 37.*0.5*(1-cos(PI*(t_thread)/(t2/2.)));
                }
                qtRZ(0, temp1des_qPEL_4x1);
                qtRY(PELx*D2R, temp4des_qPEL_4x1);
                qtRX(0, temp2des_qPEL_4x1);

                QTcross(temp1des_qPEL_4x1,temp4des_qPEL_4x1,temp3des_qPEL_4x1);
                QTcross(temp3des_qPEL_4x1,temp2des_qPEL_4x1,temp5des_qPEL_4x1);

                des_qPEL_4x1[0] = temp5des_qPEL_4x1[0];
                des_qPEL_4x1[1] = temp5des_qPEL_4x1[1];
                des_qPEL_4x1[2] = temp5des_qPEL_4x1[2];
                des_qPEL_4x1[3] = temp5des_qPEL_4x1[3];
            }
            else
            {
                des_qPEL_4x1[0]=1.;
                des_qPEL_4x1[1]=0.;
                des_qPEL_4x1[2]=0.;
                des_qPEL_4x1[3]=0.;
            }
            //================================================//

            //==============  Foot Trajectory + sagging================//
            zRF = 0.;
            if(t_thread>=t0 && t_thread<t2)
            {
                if(DemoFlag == FOOTUP)
                {
                    zc = 0.85;
                    if(t_thread>=t1+0.5 && t_thread<t1+0.8)
                    {
                        AddJoint = sagging*0.5*(1-cos(PI*(t_thread-t1-0.5)/(0.3)));
                        zLF = 0.2*0.5*(1-cos(PI*(t_thread-t1-0.5)/(t2-t1-0.5)));
                    }
                    else if(t_thread>=t1+0.8 && t_thread<t2)
                    {
                        xLF = 0.45*0.5*(1-cos(PI*(t_thread-t1-0.8)/(t2-t1-0.8)));
                        zLF = 0.2*0.5*(1-cos(PI*(t_thread-t1-0.5)/(t2-t1-0.5)));
                        LeftPitch = -30*D2R*0.5*(1-cos(PI*(t_thread-t1-0.8)/(t2-t1-0.8)));
                    }
                    else
                    {
                        xLF=zLF=LeftPitch=0;
                    }

                }
                else if(DemoFlag == KNEEDOWN)
                {
                    zc = 0.85 - 0.15*0.5*(1-cos(PI*(t_thread)/(t2-t1)));
                    if(t_thread < (t0 + (t2-t1)/1.5*4))
                    RSR_demo = 10*0.5*(1-cos(PI*(t_thread)/((t2-t1)/1.5)));
                    LSR_demo =  RSR_demo;
                    xLF = 0.45 + 0.1*0.5*(1-cos(PI*(t_thread)/(t2-t1)));
                    zLF = 0.2;
                    LeftPitch = -30*D2R;
                }
                else if(DemoFlag == FOOTDOWN)
                {
                    zc = 0.85;
                    if(t_thread<(t1-1.))
                    {
                        xLF = 0.45*(1 - 0.5*(1-cos(PI*(t_thread)/(t1-t0-1.))));
                        LeftPitch = -30*D2R*(1 - 0.5*(1-cos(PI*(t_thread)/(t1-t0-1.))));
                    }
                    if(t_thread<(t1-0.3))
                    {
                    zLF = 0.2 - (0.2-0.003)*0.5*(1-cos(PI*(t_thread)/(t1-t0-0.3)));
                    if(t_thread>=(t1-0.4))
                    AddJoint = sagging - sagging*0.5*(1-cos(PI*(t_thread-t1-0.4)/(0.1)));
                    }
                    else if(t_thread<(t1))
                    {
                    zLF = 0.003 - (0.003)*0.5*(1-cos(PI*(t_thread-t1+0.3)/(0.3)));
                    }
                }
            }
            else if(t_thread >= t2 && t_thread < (t0 + (t2-t1)/1.5*4))
            {
                if(DemoFlag == KNEEDOWN)
                {
                RSR_demo = 10*0.5*(1-cos(PI*(t_thread)/((t2-t1)/1.5)));
                LSR_demo =  RSR_demo;
                }
            }
            else
            {
                if(DemoFlag == FOOTUP)
                {
                    printf("End of Foot Up(Ready for Knee Down)!!!!!!!!!!!!!!!!!!\n");
                }
                else if(DemoFlag == KNEEDOWN)
                {
                    printf("End of Knee Down!!!!!!!!!!!!!\n");
                }
                else if(DemoFlag == FOOTDOWN)
                {
                    printf("End of Foot Down!!!!!!!!!!\n");
                }
                else if(DemoFlag == BOW)
                {
                    Qub[idRSP] = 40.*D2R;
                    Qub[idLSP] = 40.*D2R;

                    Qub[idRSR] = 10.*D2R;
                    Qub[idLSR] = -10.*D2R;

                    Qub[idRSY] = 0.*D2R;
                    Qub[idLSY] = 0.*D2R;

                    Qub[idREB] = -130.*D2R;
                    Qub[idLEB] = -130.*D2R;

                    Qub[idRWY] = 0.*D2R;
                    Qub[idLWY] = 0.*D2R;

                    Qub[idRWP] = 20.*D2R;
                    Qub[idLWP] = 20.*D2R;
                    printf("End of BOW!!!!!!!!!!\n");
                }

                infinityTest = false;
            }

            if(DemoFlag == BOW ||DemoFlag == HIT_HIT|| DemoFlag == HIT_RETURN|| DemoFlag == HIT_READY)
            {
                zc = 0.77;
                zRF = zLF = LeftPitch = RSR_demo = LSR_demo = 0.;
            }
            //==============================================//


            //==============  Control ================//
            if(DemoFlag == KNEEDOWN)
            {
//                        if(t_thread>=t1+0.5)
//                        Kirk_Control_ssp();
//                        else
//                        Kirk_Control();
            }
            else if(DemoFlag == KNEEDOWN)
            {
//                        Kirk_Control_ssp();
            }
            else if(DemoFlag == FOOTDOWN)
            {
                if(t_thread<(t1-0.5))
                {
//                            Kirk_Control_ssp();
                }
                else
                    Kirk_Control();
            }
            else if(DemoFlag == BOW || DemoFlag == HIT_HIT|| DemoFlag == HIT_READY|| DemoFlag == HIT_RETURN)
            {
                Kirk_Control();
            }
            //========================================//

            des_pCOM_3x1[0] = 0. - Del_PC_X_DSP_XZMP_CON*0.001- Del_PC_X_SSP_XZMP_CON*0.001*0.5;
            des_pCOM_3x1[1] = yc - Del_PC_Y_DSP_YZMP_CON*0.001- Del_PC_Y_SSP_YZMP_CON*0.001*0.5;
            des_pCOM_3x1[2] = zc;

            des_pRF_3x1[0] = 0.;
            des_pRF_3x1[1] = -kine_drc_hubo3.L_PEL2PEL/2.;
            des_pRF_3x1[2] = zRF;


            des_pLF_3x1[0] = xLF;
            des_pLF_3x1[1] = kine_drc_hubo3.L_PEL2PEL/2.;
            des_pLF_3x1[2] = zLF;


            qtRZ(0, temp1des_qLF_4x1);
            qtRY(LeftPitch, temp4des_qLF_4x1);
            qtRX(0, temp2des_qLF_4x1);

            QTcross(temp1des_qLF_4x1,temp4des_qLF_4x1,temp3des_qLF_4x1);
            QTcross(temp3des_qLF_4x1,temp2des_qLF_4x1,temp5des_qLF_4x1);

            des_qLF_4x1[0] = temp5des_qLF_4x1[0];
            des_qLF_4x1[1] = temp5des_qLF_4x1[1];
            des_qLF_4x1[2] = temp5des_qLF_4x1[2];
            des_qLF_4x1[3] = temp5des_qLF_4x1[3];

            t_thread += RT_TIMER_PERIOD_MS*0.001f;

            memcpy(WBIK_Q0,WBIK_Q,34*sizeof(double));

//            if(DemoFlag == HIT_HIT||DemoFlag == HIT_READY||DemoFlag == HIT_RETURN)
            {
                    Qub[idRSP] = joint->Joints[RSP]->RefAngleCurrent*D2R;
                    Qub[idLSP] = joint->Joints[LSP]->RefAngleCurrent*D2R;

                    Qub[idRSR] = (joint->Joints[RSR]->RefAngleCurrent +OFFSET_RSR)*D2R;
                    Qub[idLSR] = (joint->Joints[LSR]->RefAngleCurrent +OFFSET_LSR)*D2R;
                    Qub[idRSY] = joint->Joints[RSY]->RefAngleCurrent*D2R;
                    Qub[idLSY] = joint->Joints[LSY]->RefAngleCurrent*D2R;

                    Qub[idREB] = (joint->Joints[REB]->RefAngleCurrent + OFFSET_ELB)*D2R;
                    Qub[idLEB] = (joint->Joints[LEB]->RefAngleCurrent + OFFSET_ELB)*D2R;

                    Qub[idRWY] = joint->Joints[RWY]->RefAngleCurrent*D2R;
                    Qub[idLWY] = joint->Joints[LWY]->RefAngleCurrent*D2R;

                    Qub[idRWP] = joint->Joints[RWP]->RefAngleCurrent*D2R;
                    Qub[idLWP] = joint->Joints[LWP]->RefAngleCurrent*D2R;

                    Qub[idWST] = joint->Joints[WST]->RefAngleCurrent*D2R;
            }
            kine_drc_hubo3.IK_LowerBody_Global(WBIK_Q0,Qub,des_pCOM_3x1, des_qPEL_4x1, des_pRF_3x1, des_qRF_4x1, des_pLF_3x1, des_qLF_4x1,WBIK_Q);
            for(int i=0; i<=LAR; i++)
            {
                if(i==RAR)
                    joint->Joints[i]->RefAngleCurrent = WBIK_Q[i+7]*R2D - AddJoint;
                else
                joint->Joints[i]->RefAngleCurrent = WBIK_Q[i+7]*R2D;
            }
            if(DemoFlag == KNEEDOWN)
            {
            joint->Joints[RSR]->RefAngleCurrent =  curRSR + RSR_demo;
            joint->Joints[LSR]->RefAngleCurrent =  curLSR + LSR_demo;
            }
        }

        joint->MoveAllJoint();
        cntforcountsleep++;

        suspendThread();
	}
}
// --------------------------------------------------------------------------------------------- //
void *RBFlagThread(void *)
{
    while(isTerminated == 0)
    {
        usleep(300);
        if(HasAnyOwnership()){
            if(sharedCMD->SYNC_SIGNAL[PODO_NO] == true){
                joint->JointUpdate();
                resumeThread();
            }
        }
    }
}
// --------------------------------------------------------------------------------------------- //


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
	switch(_signal)
	{
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
int RBInitialize(void){
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
            sharedREF = (pRBCORE_SHM_REFERENCE)mmap(0, sizeof(RBCORE_SHM_REFERENCE), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
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
            sharedSEN = (pRBCORE_SHM_SENSOR)mmap(0, sizeof(RBCORE_SHM_SENSOR), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
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
// --------------------------------------------------------------------------------------------- //

void WBIK_PARA_CHANGE(){
    // NONE
    //hyoin robot
//    kine_drc_hubo3.C_Torso[0] = -0.035-0.015;
//    kine_drc_hubo3.C_Torso[1] =  0;
//    kine_drc_hubo3.m_Torso = 28.6+1;
//    kine_drc_hubo3.m_RightWrist = 3.7 ;
//    kine_drc_hubo3.m_LeftWrist = 3.7 ;

    //jungwoo robot
    kine_drc_hubo3.C_Torso[0] = 0.000941-0.055;//-0.02;
    kine_drc_hubo3.C_Torso[1] =  0;
    kine_drc_hubo3.m_Torso = 24.98723;
    kine_drc_hubo3.m_RightWrist = 4.5;
    kine_drc_hubo3.m_LeftWrist = 4.5;
    kine_drc_hubo3.m_Pelvis = 11.867886 +2.;
    kine_drc_hubo.L_FOOT = 0.113;
}
void DemoReady()
{
    WBIK_PARA_CHANGE();
    infinityTest= false;
    kine_drc_hubo.C_Torso[1] =0;
    joint->RefreshToCurrentReference();
    joint->SetAllMotionOwner();

    double postime = 300.0;


    double des_pCOM_3x1[3], des_pPCz, des_qPEL_4x1[4], des_pRF_3x1[3], des_qRF_4x1[4], des_pLF_3x1[3], des_qLF_4x1[4];

//    WBIK_PARA_CHANGE();

    des_pCOM_3x1[0] = userData->WalkReadyCOM[0] = 0.0;//0.0237f;
    des_pCOM_3x1[1] = userData->WalkReadyCOM[1] = 0.0;
    des_pCOM_3x1[2] = userData->WalkReadyCOM[2] = 0.77;//0.77;// + 0.11;//(61.8kg:0.74)//59//0.8;//71kg

    des_qPEL_4x1[0] = 1.;
    des_qPEL_4x1[1] = 0.;
    des_qPEL_4x1[2] = 0.;
    des_qPEL_4x1[3] = 0.;

    des_pRF_3x1[0] = 0.;
    des_pRF_3x1[1] = -kine_drc_hubo.L_PEL2PEL/2.;//-0.13;//-kine_drc_hubo.L_PEL2PEL/2;//-0.135;//
    des_pRF_3x1[2] = 0.;

    des_qRF_4x1[0] = 1.;
    des_qRF_4x1[1] = 0.;
    des_qRF_4x1[2] = 0.;
    des_qRF_4x1[3] = 0.;

    des_pLF_3x1[0] = 0.;
    des_pLF_3x1[1] = kine_drc_hubo.L_PEL2PEL/2.;//0.13;//kine_drc_hubo.L_PEL2PEL/2;//0.135;//
    des_pLF_3x1[2] = 0.;

    des_qLF_4x1[0] = 1.;
    des_qLF_4x1[1] = 0.;
    des_qLF_4x1[2] = 0.;
    des_qLF_4x1[3] = 0.;

    Qub[idRSP] = 40.*D2R;
    Qub[idLSP] = 40.*D2R;

    Qub[idRSR] = (10.+ OFFSET_RSR)*D2R;
    Qub[idLSR] = (-10.+ OFFSET_LSR)*D2R;

    Qub[idRSY] = 0.*D2R;
    Qub[idLSY] = 0.*D2R;

    Qub[idREB] = (-130. + OFFSET_ELB)*D2R;
    Qub[idLEB] = (-130. + OFFSET_ELB)*D2R;

    Qub[idRWY] = 0.*D2R;
    Qub[idLWY] = 0.*D2R;

    Qub[idRWP] = 20.*D2R;
    Qub[idLWP] = 20.*D2R;

    WBIK_Q0[idRHY] = 0.;
    WBIK_Q0[idRHR] = -2.78*D2R;
    WBIK_Q0[idRHP] = -43.9*D2R;
    WBIK_Q0[idRKN] = 80.58*D2R; //77.4*D2R;
    WBIK_Q0[idRAP] = -36.68*D2R;
    WBIK_Q0[idRAR] = 2.78*D2R;

    WBIK_Q0[idLHY] = 0.;
    WBIK_Q0[idLHR] = 2.78*D2R;
    WBIK_Q0[idLHP] = -43.9*D2R;
    WBIK_Q0[idLKN] = 80.58*D2R;//77.4*D2R;
    WBIK_Q0[idLAP] = -36.68*D2R;
    WBIK_Q0[idLAR] = -2.78*D2R;
    WBIK_Q0[idRSP] = 40.*D2R;
    WBIK_Q0[idLSP] = 40.*D2R;

    WBIK_Q0[idRSR] = 10.*D2R;
    WBIK_Q0[idLSR] = -10.*D2R;

    WBIK_Q0[idRSY] = 0.*D2R;
    WBIK_Q0[idLSY] = 0.*D2R;

    WBIK_Q0[idREB] = -130.*D2R;
    WBIK_Q0[idLEB] = -130.*D2R;

    WBIK_Q0[idRWY] = 0.*D2R;
    WBIK_Q0[idLWY] = 0.*D2R;

    WBIK_Q0[idRWP] = 20.*D2R;
    WBIK_Q0[idLWP] = 20.*D2R;
    WBIK_Q0[idRWY2] = 0;//20.*D2R;
    WBIK_Q0[idLWY2] = 0;//20.*D2R;
    WBIK_Q0[idWST] = 0;//-180*D2R;//20.*D2R;
//    (const double Q_ub_34x1[], const double des_pCOM_3x1[], const double des_qPEL_4x1[],
//                                const double des_pRF_3x1[], const double des_qRF_4x1[],
//                                const double des_pLF_3x1[], const double des_qLF_4x1[],
//                                double Q_return_34x1[]);
    kine_drc_hubo3.IK_LowerBody_Global(WBIK_Q0,Qub,des_pCOM_3x1, des_qPEL_4x1, des_pRF_3x1, des_qRF_4x1, des_pLF_3x1, des_qLF_4x1,WBIK_Q);


    joint->SetMoveJoint(RHY, WBIK_Q[idRHY]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHR, WBIK_Q[idRHR]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHP, WBIK_Q[idRHP]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RKN, WBIK_Q[idRKN]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RAP, WBIK_Q[idRAP]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RAR, WBIK_Q[idRAR]*R2D, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LHY, WBIK_Q[idLHY]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHR, WBIK_Q[idLHR]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHP, WBIK_Q[idLHP]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LKN, WBIK_Q[idLKN]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LAP, WBIK_Q[idLAP]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LAR, WBIK_Q[idLAR]*R2D, postime, MOVE_ABSOLUTE);
}

void GotoWalkReadyPos(){
    double postime = 3000.0;

    joint->SetMoveJoint(RSP, 40.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, 10.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -130, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, 20.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LSP, 40.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, -10.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -130.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, 20.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RSP, 40.0-120, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RSR, 10-40.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RSY, 0.0+90, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(REB, -130+45, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RWY, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RWP, 20.0, postime, MOVE_ABSOLUTE);

//    joint->SetMoveJoint(LSP, 40.0-120, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LSR, -10.0+40, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LSY, 0.0-90, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LEB, -130.0+45, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LWY, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LWP, 20.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(WST, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(NKY, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(NK1, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(NK2, 0.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RWY2, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHAND, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RF3, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY2, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHAND, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LF3, 0.0, postime, MOVE_ABSOLUTE);

    double des_pCOM_3x1[3], des_pPCz, des_qPEL_4x1[4], des_pRF_3x1[3], des_qRF_4x1[4], des_pLF_3x1[3], des_qLF_4x1[4];

    WBIK_PARA_CHANGE();

    des_pCOM_3x1[0] = userData->WalkReadyCOM[0] = 0.0;//0.0237f;
    des_pCOM_3x1[1] = userData->WalkReadyCOM[1] = 0.0;
    des_pCOM_3x1[2] = userData->WalkReadyCOM[2] = 0.77;//0.77;// + 0.11;//(61.8kg:0.74)//59//0.8;//71kg

    des_qPEL_4x1[0] = 1.;
    des_qPEL_4x1[1] = 0.;
    des_qPEL_4x1[2] = 0.;
    des_qPEL_4x1[3] = 0.;

    des_pRF_3x1[0] = 0.;
    des_pRF_3x1[1] = -kine_drc_hubo.L_PEL2PEL/2.;//-0.13;//-kine_drc_hubo.L_PEL2PEL/2;//-0.135;//
    des_pRF_3x1[2] = 0.;

    des_qRF_4x1[0] = 1.;
    des_qRF_4x1[1] = 0.;
    des_qRF_4x1[2] = 0.;
    des_qRF_4x1[3] = 0.;

    des_pLF_3x1[0] = 0.;
    des_pLF_3x1[1] = kine_drc_hubo.L_PEL2PEL/2.;//0.13;//kine_drc_hubo.L_PEL2PEL/2;//0.135;//
    des_pLF_3x1[2] = 0.;

    des_qLF_4x1[0] = 1.;
    des_qLF_4x1[1] = 0.;
    des_qLF_4x1[2] = 0.;
    des_qLF_4x1[3] = 0.;

    Qub[idRSP] = 40.*D2R;
    Qub[idLSP] = 40.*D2R;

//    Qub[idRSR] = (10.+ OFFSET_RSR)*D2R;
//    Qub[idLSR] = (-10.+ OFFSET_LSR)*D2R;
    Qub[idRSR] = (10.)*D2R;
    Qub[idLSR] = (-10.)*D2R;

    Qub[idRSY] = 0.*D2R;
    Qub[idLSY] = 0.*D2R;

//    Qub[idREB] = (-130. + OFFSET_ELB)*D2R;
//    Qub[idLEB] = (-130. + OFFSET_ELB)*D2R;
    Qub[idREB] = (-130. )*D2R;
    Qub[idLEB] = (-130. )*D2R;

    Qub[idRWY] = 0.*D2R;
    Qub[idLWY] = 0.*D2R;

    Qub[idRWP] = 20.*D2R;
    Qub[idLWP] = 20.*D2R;

    WBIK_Q0[idRHY] = 0.;
    WBIK_Q0[idRHR] = -2.78*D2R;
    WBIK_Q0[idRHP] = -43.9*D2R;
    WBIK_Q0[idRKN] = 80.58*D2R; //77.4*D2R;
    WBIK_Q0[idRAP] = -36.68*D2R;
    WBIK_Q0[idRAR] = 2.78*D2R;

    WBIK_Q0[idLHY] = 0.;
    WBIK_Q0[idLHR] = 2.78*D2R;
    WBIK_Q0[idLHP] = -43.9*D2R;
    WBIK_Q0[idLKN] = 80.58*D2R;//77.4*D2R;
    WBIK_Q0[idLAP] = -36.68*D2R;
    WBIK_Q0[idLAR] = -2.78*D2R;
    WBIK_Q0[idRSP] = 40.*D2R;
    WBIK_Q0[idLSP] = 40.*D2R;

    WBIK_Q0[idRSR] = 10.*D2R;
    WBIK_Q0[idLSR] = -10.*D2R;

    WBIK_Q0[idRSY] = 0.*D2R;
    WBIK_Q0[idLSY] = 0.*D2R;

    WBIK_Q0[idREB] = -130.*D2R;
    WBIK_Q0[idLEB] = -130.*D2R;

    WBIK_Q0[idRWY] = 0.*D2R;
    WBIK_Q0[idLWY] = 0.*D2R;

    WBIK_Q0[idRWP] = 20.*D2R;
    WBIK_Q0[idLWP] = 20.*D2R;
    WBIK_Q0[idRWY2] = 0;//20.*D2R;
    WBIK_Q0[idLWY2] = 0;//20.*D2R;
    WBIK_Q0[idWST] = 0;//-180*D2R;//20.*D2R;
//    (const double Q_ub_34x1[], const double des_pCOM_3x1[], const double des_qPEL_4x1[],
//                                const double des_pRF_3x1[], const double des_qRF_4x1[],
//                                const double des_pLF_3x1[], const double des_qLF_4x1[],
//                                double Q_return_34x1[]);
    kine_drc_hubo3.IK_LowerBody_Global(WBIK_Q0,Qub,des_pCOM_3x1, des_qPEL_4x1, des_pRF_3x1, des_qRF_4x1, des_pLF_3x1, des_qLF_4x1,WBIK_Q);

    joint->SetMoveJoint(RHY, WBIK_Q[idRHY]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHR, WBIK_Q[idRHR]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHP, WBIK_Q[idRHP]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RKN, WBIK_Q[idRKN]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RAP, WBIK_Q[idRAP]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RAR, WBIK_Q[idRAR]*R2D, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LHY, WBIK_Q[idLHY]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHR, WBIK_Q[idLHR]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHP, WBIK_Q[idLHP]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LKN, WBIK_Q[idLKN]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LAP, WBIK_Q[idLAP]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LAR, WBIK_Q[idLAR]*R2D, postime, MOVE_ABSOLUTE);

    double total_mass;
   double total_lower_mass;
   total_mass          = kine_drc_hubo.m_Pelvis + kine_drc_hubo.m_Torso + kine_drc_hubo.m_LeftUpperLeg+kine_drc_hubo.m_RightUpperLeg+kine_drc_hubo.m_LeftLowerLeg+kine_drc_hubo.m_RightLowerLeg+kine_drc_hubo.m_LeftFoot+kine_drc_hubo.m_RightFoot+kine_drc_hubo.m_LeftLowerArm+kine_drc_hubo.m_RightLowerArm+kine_drc_hubo.m_LeftUpperArm+kine_drc_hubo.m_RightUpperArm + kine_drc_hubo.m_LeftHand + kine_drc_hubo.m_RightHand + kine_drc_hubo.m_LeftWrist+kine_drc_hubo.m_RightWrist;
   total_lower_mass    = kine_drc_hubo.m_Pelvis + kine_drc_hubo.m_LeftUpperLeg+kine_drc_hubo.m_RightUpperLeg+kine_drc_hubo.m_LeftLowerLeg+kine_drc_hubo.m_RightLowerLeg+kine_drc_hubo.m_LeftFoot+kine_drc_hubo.m_RightFoot;
   printf("total lower mass : %f\n",total_lower_mass);
   printf("total mass : %f\n",total_mass);
}
void GotoWalkReadyPos_with_WST_M180(){

    //sharedData->STATE_COMMAND = TCMD_TERRAIN_180_WALKREADY_START;

    double postime = 5000.0;

    joint->SetMoveJoint(RSP, 60.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, 15.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -120, postime-3000., MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, -85.0, postime-3000., MOVE_ABSOLUTE);

    joint->SetMoveJoint(LSP, 60.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, -15.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -120.0, postime-3000., MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, -85.0, postime-3000., MOVE_ABSOLUTE);

    joint->SetMoveJoint(WST, -180.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(WST, 0.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RWY2, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHAND, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY2, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHAND, 0.0, postime, MOVE_ABSOLUTE);

    double des_pCOM_3x1[3], des_pPCz, des_qPEL_4x1[4], des_pRF_3x1[3], des_qRF_4x1[4], des_pLF_3x1[3], des_qLF_4x1[4];

    WBIK_PARA_CHANGE();

    des_pCOM_3x1[0] = userData->WalkReadyCOM[0] = 0.0;//0.0237f;
    des_pCOM_3x1[1] = userData->WalkReadyCOM[1] = 0;//-kine_drc_hubo.L_PEL2PEL/2.;
    des_pCOM_3x1[2] = userData->WalkReadyCOM[2] = 0.77;// + 0.11;//(61.8kg:0.74)//59//0.8;//71kg

    des_qPEL_4x1[0] = 1.;
    des_qPEL_4x1[1] = 0.;
    des_qPEL_4x1[2] = 0.;
    des_qPEL_4x1[3] = 0.;

    des_pRF_3x1[0] = 0.;
    des_pRF_3x1[1] = -kine_drc_hubo.L_PEL2PEL/2.;//-0.13;//-kine_drc_hubo.L_PEL2PEL/2;//-0.135;//
    des_pRF_3x1[2] = 0.;

    des_qRF_4x1[0] = 1.;
    des_qRF_4x1[1] = 0.;
    des_qRF_4x1[2] = 0.;
    des_qRF_4x1[3] = 0.;

    des_pLF_3x1[0] = 0.;
    des_pLF_3x1[1] = kine_drc_hubo.L_PEL2PEL/2.;//0.13;//kine_drc_hubo.L_PEL2PEL/2;//0.135;//
    des_pLF_3x1[2] = 0;//0.08;

    des_qLF_4x1[0] = 1.;
    des_qLF_4x1[1] = 0.;
    des_qLF_4x1[2] = 0.;
    des_qLF_4x1[3] = 0.;

    Qub[idRSP] = 60.*D2R;//-60.*D2R;
    Qub[idLSP] = 60.*D2R;//-60.*D2R;

    Qub[idRSR] = 10.*D2R;
    Qub[idLSR] = -10.*D2R;

    Qub[idRSY] = 0.*D2R;
    Qub[idLSY] = 0.*D2R;

    Qub[idREB] = -120.*D2R;
    Qub[idLEB] = -120.*D2R;

    Qub[idRWY] = 0.*D2R;
    Qub[idLWY] = 0.*D2R;

    Qub[idRWP] = -90.*D2R;
    Qub[idLWP] = -90.*D2R;
    Qub[idWST] = -180.*D2R;
//    Qub[idWST] = 0.*D2R;

    WBIK_Q0[idRHY] = 0.;
    WBIK_Q0[idRHR] = -2.78*D2R;
    WBIK_Q0[idRHP] = -43.9*D2R;
    WBIK_Q0[idRKN] = 80.58*D2R; //77.4*D2R;
    WBIK_Q0[idRAP] = -36.68*D2R;
    WBIK_Q0[idRAR] = 2.78*D2R;

    WBIK_Q0[idLHY] = 0.;
    WBIK_Q0[idLHR] = 2.78*D2R;
    WBIK_Q0[idLHP] = -43.9*D2R;
    WBIK_Q0[idLKN] = 80.58*D2R;//77.4*D2R;
    WBIK_Q0[idLAP] = -36.68*D2R;
    WBIK_Q0[idLAR] = -2.78*D2R;
    WBIK_Q0[idRSP] = 40.*D2R;
    WBIK_Q0[idLSP] = 40.*D2R;

    WBIK_Q0[idRSR] = 10.*D2R;
    WBIK_Q0[idLSR] = -10.*D2R;

    WBIK_Q0[idRSY] = 0.*D2R;
    WBIK_Q0[idLSY] = 0.*D2R;

    WBIK_Q0[idREB] = -130.*D2R;
    WBIK_Q0[idLEB] = -130.*D2R;

    WBIK_Q0[idRWY] = 0.*D2R;
    WBIK_Q0[idLWY] = 0.*D2R;

    WBIK_Q0[idRWP] = 20.*D2R;
    WBIK_Q0[idLWP] = 20.*D2R;
    WBIK_Q0[idRWY2] = 0;//20.*D2R;
    WBIK_Q0[idLWY2] = 0;//20.*D2R;
    WBIK_Q0[idWST] = -180*D2R;//20.*D2R;
//    (const double Q_ub_34x1[], const double des_pCOM_3x1[], const double des_qPEL_4x1[],
//                                const double des_pRF_3x1[], const double des_qRF_4x1[],
//                                const double des_pLF_3x1[], const double des_qLF_4x1[],
//                                double Q_return_34x1[]);
    kine_drc_hubo3.IK_LowerBody_Global(WBIK_Q0,Qub,des_pCOM_3x1, des_qPEL_4x1, des_pRF_3x1, des_qRF_4x1, des_pLF_3x1, des_qLF_4x1,WBIK_Q);

    joint->SetMoveJoint(RHY, WBIK_Q[idRHY]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHR, WBIK_Q[idRHR]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHP, WBIK_Q[idRHP]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RKN, WBIK_Q[idRKN]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RAP, WBIK_Q[idRAP]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RAR, WBIK_Q[idRAR]*R2D, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LHY, WBIK_Q[idLHY]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHR, WBIK_Q[idLHR]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHP, WBIK_Q[idLHP]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LKN, WBIK_Q[idLKN]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LAP, WBIK_Q[idLAP]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LAR, WBIK_Q[idLAR]*R2D, postime, MOVE_ABSOLUTE);

    double total_mass;
   double total_lower_mass;
   total_mass          = kine_drc_hubo.m_Pelvis + kine_drc_hubo.m_Torso + kine_drc_hubo.m_LeftUpperLeg+kine_drc_hubo.m_RightUpperLeg+kine_drc_hubo.m_LeftLowerLeg+kine_drc_hubo.m_RightLowerLeg+kine_drc_hubo.m_LeftFoot+kine_drc_hubo.m_RightFoot+kine_drc_hubo.m_LeftLowerArm+kine_drc_hubo.m_RightLowerArm+kine_drc_hubo.m_LeftUpperArm+kine_drc_hubo.m_RightUpperArm + kine_drc_hubo.m_LeftHand + kine_drc_hubo.m_RightHand + kine_drc_hubo.m_LeftWrist+kine_drc_hubo.m_RightWrist;
   total_lower_mass    = kine_drc_hubo.m_Pelvis + kine_drc_hubo.m_LeftUpperLeg+kine_drc_hubo.m_RightUpperLeg+kine_drc_hubo.m_LeftLowerLeg+kine_drc_hubo.m_RightLowerLeg+kine_drc_hubo.m_LeftFoot+kine_drc_hubo.m_RightFoot;
   printf("total lower mass : %f\n",total_lower_mass);
   printf("total mass : %f\n",total_mass);

   usleep((postime+15)*1000);





   //sharedData->STATE_COMMAND = TCMD_TERRAIN_180_WALKREADY_DONE;
}
void GotoWalkReadyPos4(){
    double postime = 3000.0;

    joint->SetMoveJoint(RSP, 40.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, 10.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -130, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, 20.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LSP, 40.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, -10.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -130.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, 20.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RSP, 40.0-120, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RSR, 10-40.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RSY, 0.0+90, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(REB, -130+45, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RWY, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RWP, 20.0, postime, MOVE_ABSOLUTE);

//    joint->SetMoveJoint(LSP, 40.0-120, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LSR, -10.0+40, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LSY, 0.0-90, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LEB, -130.0+45, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LWY, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LWP, 20.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(WST, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(NKY, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(NK1, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(NK2, 0.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RWY2, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHAND, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RF3, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY2, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHAND, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LF3, 0.0, postime, MOVE_ABSOLUTE);

    double des_pCOM_3x1[3], des_pPCz, des_qPEL_4x1[4], des_pRF_3x1[3], des_qRF_4x1[4], des_pLF_3x1[3], des_qLF_4x1[4];

    WBIK_PARA_CHANGE();

    des_pCOM_3x1[0] = userData->WalkReadyCOM[0] = 0.0;//0.0237f;
    des_pCOM_3x1[1] = userData->WalkReadyCOM[1] = 0.0;
    des_pCOM_3x1[2] = userData->WalkReadyCOM[2] = 0.77;//0.77;// + 0.11;//(61.8kg:0.74)//59//0.8;//71kg

    des_qPEL_4x1[0] = 1.;
    des_qPEL_4x1[1] = 0.;
    des_qPEL_4x1[2] = 0.;
    des_qPEL_4x1[3] = 0.;

    des_pRF_3x1[0] = 0.;
    des_pRF_3x1[1] = -kine_drc_hubo.L_PEL2PEL/2.;//-0.13;//-kine_drc_hubo.L_PEL2PEL/2;//-0.135;//
    des_pRF_3x1[2] = 0.;

    des_qRF_4x1[0] = 1.;
    des_qRF_4x1[1] = 0.;
    des_qRF_4x1[2] = 0.;
    des_qRF_4x1[3] = 0.;

    des_pLF_3x1[0] = 0.;
    des_pLF_3x1[1] = kine_drc_hubo.L_PEL2PEL/2.;//0.13;//kine_drc_hubo.L_PEL2PEL/2;//0.135;//
    des_pLF_3x1[2] = 0.;

    des_qLF_4x1[0] = 1.;
    des_qLF_4x1[1] = 0.;
    des_qLF_4x1[2] = 0.;
    des_qLF_4x1[3] = 0.;

    Qub[idRSP] = 40.*D2R;
    Qub[idLSP] = 40.*D2R;

    Qub[idRSR] = (10.+ OFFSET_RSR)*D2R;
    Qub[idLSR] = (-10.+ OFFSET_LSR)*D2R;
//    Qub[idRSR] = (10.)*D2R;
//    Qub[idLSR] = (-10.)*D2R;

    Qub[idRSY] = 0.*D2R;
    Qub[idLSY] = 0.*D2R;

    Qub[idREB] = (-130. + OFFSET_ELB)*D2R;
    Qub[idLEB] = (-130. + OFFSET_ELB)*D2R;
//    Qub[idREB] = (-130. )*D2R;
//    Qub[idLEB] = (-130. )*D2R;

    Qub[idRWY] = 0.*D2R;
    Qub[idLWY] = 0.*D2R;

    Qub[idRWP] = 20.*D2R;
    Qub[idLWP] = 20.*D2R;

    WBIK_Q0[idRHY] = 0.;
    WBIK_Q0[idRHR] = -2.78*D2R;
    WBIK_Q0[idRHP] = -43.9*D2R;
    WBIK_Q0[idRKN] = 80.58*D2R; //77.4*D2R;
    WBIK_Q0[idRAP] = -36.68*D2R;
    WBIK_Q0[idRAR] = 2.78*D2R;

    WBIK_Q0[idLHY] = 0.;
    WBIK_Q0[idLHR] = 2.78*D2R;
    WBIK_Q0[idLHP] = -43.9*D2R;
    WBIK_Q0[idLKN] = 80.58*D2R;//77.4*D2R;
    WBIK_Q0[idLAP] = -36.68*D2R;
    WBIK_Q0[idLAR] = -2.78*D2R;
    WBIK_Q0[idRSP] = 40.*D2R;
    WBIK_Q0[idLSP] = 40.*D2R;

    WBIK_Q0[idRSR] = 10.*D2R;
    WBIK_Q0[idLSR] = -10.*D2R;

    WBIK_Q0[idRSY] = 0.*D2R;
    WBIK_Q0[idLSY] = 0.*D2R;

    WBIK_Q0[idREB] = -130.*D2R;
    WBIK_Q0[idLEB] = -130.*D2R;

    WBIK_Q0[idRWY] = 0.*D2R;
    WBIK_Q0[idLWY] = 0.*D2R;

    WBIK_Q0[idRWP] = 20.*D2R;
    WBIK_Q0[idLWP] = 20.*D2R;
    WBIK_Q0[idRWY2] = 0;//20.*D2R;
    WBIK_Q0[idLWY2] = 0;//20.*D2R;
    WBIK_Q0[idWST] = 0;//-180*D2R;//20.*D2R;
//    (const double Q_ub_34x1[], const double des_pCOM_3x1[], const double des_qPEL_4x1[],
//                                const double des_pRF_3x1[], const double des_qRF_4x1[],
//                                const double des_pLF_3x1[], const double des_qLF_4x1[],
//                                double Q_return_34x1[]);
    kine_drc_hubo3.IK_LowerBody_Global(WBIK_Q0,Qub,des_pCOM_3x1, des_qPEL_4x1, des_pRF_3x1, des_qRF_4x1, des_pLF_3x1, des_qLF_4x1,WBIK_Q);

    joint->SetMoveJoint(RHY, WBIK_Q[idRHY]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHR, WBIK_Q[idRHR]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHP, WBIK_Q[idRHP]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RKN, WBIK_Q[idRKN]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RAP, WBIK_Q[idRAP]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RAR, WBIK_Q[idRAR]*R2D, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LHY, WBIK_Q[idLHY]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHR, WBIK_Q[idLHR]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHP, WBIK_Q[idLHP]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LKN, WBIK_Q[idLKN]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LAP, WBIK_Q[idLAP]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LAR, WBIK_Q[idLAR]*R2D, postime, MOVE_ABSOLUTE);

    double total_mass;
   double total_lower_mass;
   total_mass          = kine_drc_hubo.m_Pelvis + kine_drc_hubo.m_Torso + kine_drc_hubo.m_LeftUpperLeg+kine_drc_hubo.m_RightUpperLeg+kine_drc_hubo.m_LeftLowerLeg+kine_drc_hubo.m_RightLowerLeg+kine_drc_hubo.m_LeftFoot+kine_drc_hubo.m_RightFoot+kine_drc_hubo.m_LeftLowerArm+kine_drc_hubo.m_RightLowerArm+kine_drc_hubo.m_LeftUpperArm+kine_drc_hubo.m_RightUpperArm + kine_drc_hubo.m_LeftHand + kine_drc_hubo.m_RightHand + kine_drc_hubo.m_LeftWrist+kine_drc_hubo.m_RightWrist;
   total_lower_mass    = kine_drc_hubo.m_Pelvis + kine_drc_hubo.m_LeftUpperLeg+kine_drc_hubo.m_RightUpperLeg+kine_drc_hubo.m_LeftLowerLeg+kine_drc_hubo.m_RightLowerLeg+kine_drc_hubo.m_LeftFoot+kine_drc_hubo.m_RightFoot;
   printf("total lower mass : %f\n",total_lower_mass);
   printf("total mass : %f\n",total_mass);
}
void GotoHomePos(){
    double postime = 3000.0;
    for(int i=0; i<NO_OF_JOINTS; i++)
        joint->SetMoveJoint(i, 0.0, postime, MOVE_ABSOLUTE);
}
void GotoDrillWidePos(){
    double postime = 6000.0;

    joint->SetMoveJoint(RSP, 10.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, -10.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -135.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, 20.0, postime, MOVE_ABSOLUTE);

//    joint->SetMoveJoint(LSP, 10.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LSR, 10.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LEB, -135.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LWY, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LWP, 20.0, postime, MOVE_ABSOLUTE);


    joint->SetMoveJoint(LSP, 60.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, -10.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -130.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, -40.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(WST, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(NKY, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(NK1, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(NK2, 0.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RWY2, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHAND, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RF3, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY2, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHAND, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LF3, 0.0, postime, MOVE_ABSOLUTE);

    double des_pCOM_3x1[3], des_pPCz, des_qPEL_4x1[4], des_pRF_3x1[3], des_qRF_4x1[4], des_pLF_3x1[3], des_qLF_4x1[4];

    WBIK_PARA_CHANGE();


    //printf("foot mass = %f\n",kine_drc_hubo.m_LEFT_FOOT);
    //printf ("total mass = %f\n",kine_drc_hubo.m_LEFT_ANKLE*2+kine_drc_hubo.m_LEFT_FOOT*2+kine_drc_hubo.m_LEFT_LOWER_LEG*2+kine_drc_hubo.m_LEFT_UPPER_LEG*2+kine_drc_hubo.m_PELVIS+kine_drc_hubo.m_TORSO+kine_drc_hubo.m_LEFT_HAND*2+kine_drc_hubo.m_LEFT_LOWER_ARM*2+kine_drc_hubo.m_LEFT_UPPER_ARM*2);


    des_pCOM_3x1[0] = userData->WalkReadyCOM[0] = 0.0;//0.0237f;
    des_pCOM_3x1[1] = userData->WalkReadyCOM[1] = 0.0;
    //des_pCOM_3x1[2] = userData->WalkReadyCOM[2] = 0.71;
    des_pCOM_3x1[2] = userData->WalkReadyCOM[2] = 0.81;
    //printf("WR_COM_Z = %f\n",WR_COM_Z);
    //des_pPCz = 0.63;

    des_qPEL_4x1[0] = 1.;
    des_qPEL_4x1[1] = 0.;
    des_qPEL_4x1[2] = 0.;
    des_qPEL_4x1[3] = 0.;

    des_pRF_3x1[0] = 0.;
    des_pRF_3x1[1] = -0.2;
    des_pRF_3x1[2] = 0.;

    des_qRF_4x1[0] = 1.;
    des_qRF_4x1[1] = 0.;
    des_qRF_4x1[2] = 0.;
    des_qRF_4x1[3] = 0.;

    des_pLF_3x1[0] = 0.;
    des_pLF_3x1[1] = 0.2;
    des_pLF_3x1[2] = 0.;

    des_qLF_4x1[0] = 1.;
    des_qLF_4x1[1] = 0.;
    des_qLF_4x1[2] = 0.;
    des_qLF_4x1[3] = 0.;

    Qub[idRSP] = 10.*D2R;
    Qub[idLSP] = 10.*D2R;

    Qub[idRSR] = -10.*D2R;
    Qub[idLSR] = 10.*D2R;

    Qub[idRSY] = 0.*D2R;
    Qub[idLSY] = 0.*D2R;

    Qub[idREB] = -135.*D2R;
    Qub[idLEB] = -135.*D2R;

    Qub[idRWY] = 0.*D2R;
    Qub[idLWY] = 0.*D2R;

    Qub[idRWP] = 20.*D2R;
    Qub[idLWP] = 20.*D2R;

    kine_drc_hubo.IK_LowerBody_Global(Qub,des_pCOM_3x1, des_qPEL_4x1, des_pRF_3x1, des_qRF_4x1, des_pLF_3x1, des_qLF_4x1,WBIK_Q);

    joint->SetMoveJoint(RHY, WBIK_Q[idRHY]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHR, WBIK_Q[idRHR]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHP, WBIK_Q[idRHP]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RKN, WBIK_Q[idRKN]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RAP, WBIK_Q[idRAP]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RAR, WBIK_Q[idRAR]*R2D, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LHY, WBIK_Q[idLHY]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHR, WBIK_Q[idLHR]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHP, WBIK_Q[idLHP]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LKN, WBIK_Q[idLKN]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LAP, WBIK_Q[idLAP]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LAR, WBIK_Q[idLAR]*R2D, postime, MOVE_ABSOLUTE);
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

int RBindicator(unsigned char number)
{
//    MANUAL_CAN	MCData;

//    MCData.channel = 1;
//    MCData.id = COMMAND_CANID;
//    MCData.dlc = 3;
//    MCData.data[0] = 24;
//    MCData.data[1] = 0x84;
//    MCData.data[2] = (unsigned char)number;

//    return PushCANMessage(MCData);
}
void GotoWalkReadyPos3(){
    double postime = 3000.0;

    joint->SetMoveJoint(RSP, 40.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, 10.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -130, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, 20.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LSP, 40.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, -10.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -130.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, 20.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RSP, 40.0-120, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RSR, 10-40.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RSY, 0.0+90, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(REB, -130+45, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RWY, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RWP, 20.0, postime, MOVE_ABSOLUTE);

//    joint->SetMoveJoint(LSP, 40.0-120, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LSR, -10.0+40, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LSY, 0.0-90, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LEB, -130.0+45, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LWY, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LWP, 20.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(WST, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(NKY, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(NK1, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(NK2, 0.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RWY2, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHAND, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RF3, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY2, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHAND, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LF3, 0.0, postime, MOVE_ABSOLUTE);

    double des_pCOM_3x1[3], des_pPCz, des_qPEL_4x1[4], des_pRF_3x1[3], des_qRF_4x1[4], des_pLF_3x1[3], des_qLF_4x1[4];

    WBIK_PARA_CHANGE();

    des_pCOM_3x1[0] = userData->WalkReadyCOM[0] = 0.0;//0.0237f;
    des_pCOM_3x1[1] = userData->WalkReadyCOM[1] = 0.0;
    des_pCOM_3x1[2] = userData->WalkReadyCOM[2] = 0.85;//0.77;// + 0.11;//(61.8kg:0.74)//59//0.8;//71kg

    des_qPEL_4x1[0] = 1.;
    des_qPEL_4x1[1] = 0.;
    des_qPEL_4x1[2] = 0.;
    des_qPEL_4x1[3] = 0.;

    des_pRF_3x1[0] = 0.;
    des_pRF_3x1[1] = -kine_drc_hubo3.L_PEL2PEL/2.;//-0.13;//-kine_drc_hubo.L_PEL2PEL/2;//-0.135;//
    des_pRF_3x1[2] = 0.;

    des_qRF_4x1[0] = 1.;
    des_qRF_4x1[1] = 0.;
    des_qRF_4x1[2] = 0.;
    des_qRF_4x1[3] = 0.;

    des_pLF_3x1[0] = 0.;
    des_pLF_3x1[1] = kine_drc_hubo3.L_PEL2PEL/2.;//0.13;//kine_drc_hubo.L_PEL2PEL/2;//0.135;//
    des_pLF_3x1[2] = 0.;

    des_qLF_4x1[0] = 1.;
    des_qLF_4x1[1] = 0.;
    des_qLF_4x1[2] = 0.;
    des_qLF_4x1[3] = 0.;

    Qub[idRSP] = 40.*D2R;
    Qub[idLSP] = 40.*D2R;

    Qub[idRSR] = (10.+ OFFSET_RSR)*D2R;
    Qub[idLSR] = (-10.+ OFFSET_LSR)*D2R;
//    Qub[idRSR] = (10.)*D2R;
//    Qub[idLSR] = (-10.)*D2R;

    Qub[idRSY] = 0.*D2R;
    Qub[idLSY] = 0.*D2R;

    Qub[idREB] = (-130. + OFFSET_ELB)*D2R;
    Qub[idLEB] = (-130. + OFFSET_ELB)*D2R;
//    Qub[idREB] = (-130. )*D2R;
//    Qub[idLEB] = (-130. )*D2R;

    Qub[idRWY] = 0.*D2R;
    Qub[idLWY] = 0.*D2R;

    Qub[idRWP] = 20.*D2R;
    Qub[idLWP] = 20.*D2R;

    WBIK_Q0[idRHY] = 0.;
    WBIK_Q0[idRHR] = -2.78*D2R;
    WBIK_Q0[idRHP] = -43.9*D2R;
    WBIK_Q0[idRKN] = 80.58*D2R; //77.4*D2R;
    WBIK_Q0[idRAP] = -36.68*D2R;
    WBIK_Q0[idRAR] = 2.78*D2R;

    WBIK_Q0[idLHY] = 0.;
    WBIK_Q0[idLHR] = 2.78*D2R;
    WBIK_Q0[idLHP] = -43.9*D2R;
    WBIK_Q0[idLKN] = 80.58*D2R;//77.4*D2R;
    WBIK_Q0[idLAP] = -36.68*D2R;
    WBIK_Q0[idLAR] = -2.78*D2R;
    WBIK_Q0[idRSP] = 40.*D2R;
    WBIK_Q0[idLSP] = 40.*D2R;

    WBIK_Q0[idRSR] = 10.*D2R;
    WBIK_Q0[idLSR] = -10.*D2R;

    WBIK_Q0[idRSY] = 0.*D2R;
    WBIK_Q0[idLSY] = 0.*D2R;

    WBIK_Q0[idREB] = -130.*D2R;
    WBIK_Q0[idLEB] = -130.*D2R;

    WBIK_Q0[idRWY] = 0.*D2R;
    WBIK_Q0[idLWY] = 0.*D2R;

    WBIK_Q0[idRWP] = 20.*D2R;
    WBIK_Q0[idLWP] = 20.*D2R;
    WBIK_Q0[idRWY2] = 0;//20.*D2R;
    WBIK_Q0[idLWY2] = 0;//20.*D2R;
    WBIK_Q0[idWST] = 0;//-180*D2R;//20.*D2R;
//    (const double Q_ub_34x1[], const double des_pCOM_3x1[], const double des_qPEL_4x1[],
//                                const double des_pRF_3x1[], const double des_qRF_4x1[],
//                                const double des_pLF_3x1[], const double des_qLF_4x1[],
//                                double Q_return_34x1[]);
    kine_drc_hubo3.IK_LowerBody_Global(WBIK_Q0,Qub,des_pCOM_3x1, des_qPEL_4x1, des_pRF_3x1, des_qRF_4x1, des_pLF_3x1, des_qLF_4x1,WBIK_Q);

    joint->SetMoveJoint(RHY, WBIK_Q[idRHY]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHR, WBIK_Q[idRHR]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHP, WBIK_Q[idRHP]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RKN, WBIK_Q[idRKN]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RAP, WBIK_Q[idRAP]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RAR, WBIK_Q[idRAR]*R2D, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LHY, WBIK_Q[idLHY]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHR, WBIK_Q[idLHR]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHP, WBIK_Q[idLHP]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LKN, WBIK_Q[idLKN]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LAP, WBIK_Q[idLAP]*R2D, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LAR, WBIK_Q[idLAR]*R2D, postime, MOVE_ABSOLUTE);

    double total_mass;
   double total_lower_mass;
   total_mass          = kine_drc_hubo.m_Pelvis + kine_drc_hubo.m_Torso + kine_drc_hubo.m_LeftUpperLeg+kine_drc_hubo.m_RightUpperLeg+kine_drc_hubo.m_LeftLowerLeg+kine_drc_hubo.m_RightLowerLeg+kine_drc_hubo.m_LeftFoot+kine_drc_hubo.m_RightFoot+kine_drc_hubo.m_LeftLowerArm+kine_drc_hubo.m_RightLowerArm+kine_drc_hubo.m_LeftUpperArm+kine_drc_hubo.m_RightUpperArm + kine_drc_hubo.m_LeftHand + kine_drc_hubo.m_RightHand + kine_drc_hubo.m_LeftWrist+kine_drc_hubo.m_RightWrist;
   total_lower_mass    = kine_drc_hubo.m_Pelvis + kine_drc_hubo.m_LeftUpperLeg+kine_drc_hubo.m_RightUpperLeg+kine_drc_hubo.m_LeftLowerLeg+kine_drc_hubo.m_RightLowerLeg+kine_drc_hubo.m_LeftFoot+kine_drc_hubo.m_RightFoot;
   printf("total lower mass : %f\n",total_lower_mass);
   printf("total mass : %f\n",total_mass);
}
void WholebodyInitialize()
{
    WBIK_PARA_CHANGE();
    Qub[idRSP] = 0.*D2R;
    Qub[idRSR] = -40.*D2R;
    Qub[idRSY] = -30.*D2R;
    Qub[idREB] = -70.*D2R;

    Qub[idLSP] = 0.*D2R;
    Qub[idLSR] = 40.*D2R;
    Qub[idLSY] = 30.*D2R;
    Qub[idLEB] = -70.*D2R;

    Qub[idRWP] = 20.*D2R;
    Qub[idLWP] = 20.*D2R;

    WBIK_Q0[idRHY] = 0.;
    WBIK_Q0[idRHR] = -2.78*D2R;
    WBIK_Q0[idRHP] = -43.9*D2R;
    WBIK_Q0[idRKN] = 80.58*D2R; //77.4*D2R;
    WBIK_Q0[idRAP] = -36.68*D2R;
    WBIK_Q0[idRAR] = 2.78*D2R;

    WBIK_Q0[idLHY] = 0.;
    WBIK_Q0[idLHR] = 2.78*D2R;
    WBIK_Q0[idLHP] = -43.9*D2R;
    WBIK_Q0[idLKN] = 80.58*D2R;//77.4*D2R;
    WBIK_Q0[idLAP] = -36.68*D2R;
    WBIK_Q0[idLAR] = -2.78*D2R;
    WBIK_Q0[idRSP] = -5.*D2R;
    WBIK_Q0[idLSP] = -5.*D2R;

    WBIK_Q0[idRSR] = 10.*D2R;
    WBIK_Q0[idLSR] = -10.*D2R;

    WBIK_Q0[idRSY] = 0.*D2R;
    WBIK_Q0[idLSY] = 0.*D2R;

    WBIK_Q0[idREB] = -130.*D2R;
    WBIK_Q0[idLEB] = -130.*D2R;

    WBIK_Q0[idRWY] = 0.*D2R;
    WBIK_Q0[idLWY] = 0.*D2R;

    WBIK_Q0[idRWP] = 20.*D2R;
    WBIK_Q0[idLWP] = 20.*D2R;
    WBIK_Q0[idRWY2] = 0;//20.*D2R;
    WBIK_Q0[idLWY2] = 0;//20.*D2R;
    WBIK_Q0[idWST] = 0;//-180*D2R;//20.*D2R;
}
void get_zmp2()
{
        // ZMP 읽기 //
        double M_LF[3],M_RF[3],M_LF_Global[3],M_RF_Global[3],F_LF[3],F_RF[3],F_LF_Global[3],F_RF_Global[3];
        double pCenter[3],qCenter[4],qCenter_bar[4];
        double zmp[3],zmp_local[3],zmp_ref_local[3];

        // Foot Center in Global Coord.
        pCenter[0] = (des_pRF_3x1[0] + des_pLF_3x1[0])/2.;
        pCenter[1] = (des_pRF_3x1[1] + des_pLF_3x1[1])/2.;
        pCenter[2] = (des_pRF_3x1[2] + des_pLF_3x1[2])/2.;

//        one_cos_orientation2(0.5,des_qRF_4x1,des_qLF_4x1,0,1,qCenter);
        qtRZ(0.,qCenter);


        if(sharedSEN->FT[RAFT].Fz + sharedSEN->FT[LAFT].Fz > 50.)
        {
            M_LF[0] =  sharedSEN->FT[LAFT].Mx;
            M_LF[1] =  sharedSEN->FT[LAFT].My;
            M_LF[2] =  sharedSEN->FT[LAFT].Mz;

            QTtransform(des_qLF_4x1,M_LF,M_LF_Global);

            M_RF[0] =  sharedSEN->FT[RAFT].Mx;
            M_RF[1] =  sharedSEN->FT[RAFT].My;
            M_RF[2] =  sharedSEN->FT[RAFT].Mz;

            QTtransform(des_qRF_4x1,M_RF,M_RF_Global);

            F_LF[0] = sharedSEN->FT[LAFT].Fx;
            F_LF[1] = sharedSEN->FT[LAFT].Fy;
            F_LF[2] = sharedSEN->FT[LAFT].Fz;

            QTtransform(des_qLF_4x1,F_LF,F_LF_Global);

            F_RF[0] = sharedSEN->FT[RAFT].Fx;
            F_RF[1] = sharedSEN->FT[RAFT].Fy;
            F_RF[2] = sharedSEN->FT[RAFT].Fz;

            QTtransform(des_qRF_4x1,F_RF,F_RF_Global);

             double temp1[3],temp2[3],temp3[3],temp4[3];

             diff_vv(des_pRF_3x1,3,pCenter,temp1);// (despRF - pCenter)
             diff_vv(des_pLF_3x1,3,pCenter,temp2);// (despLF - pCenter)

             cross(1,temp1,F_RF_Global,temp3);// (despRF - pCenter)x(F_RF_Global)
             cross(1,temp2,F_LF_Global,temp4);// (despLF - pCenter)x(F_LF_Global)

             sum_vv(temp3,3,temp4,temp3); // (despRF - pCenter)x(F_RF_Global) + (despLF - pCenter)x(F_LF_Global)
             sum_vv(temp3,3,M_RF_Global,temp3); // (despRF - pCenter)x(F_RF_Global) + (despLF - pCenter)x(F_LF_Global) + M_RF_Global
             sum_vv(temp3,3,M_LF_Global,temp3); // (despRF - pCenter)x(F_RF_Global) + (despLF - pCenter)x(F_LF_Global) + M_RF_Global + M_LF_Global

             zmp[0] = (-temp3[1])/(F_RF_Global[2] + F_LF_Global[2]) + pCenter[0];
             zmp[1] = (temp3[0])/(F_RF_Global[2] + F_LF_Global[2]) + pCenter[1];
             zmp[2] = 0.;

             diff_vv(zmp,3,pCenter,temp1); // zmp - pCenter

             qCenter_bar[0] = qCenter[0];
             qCenter_bar[1] = -qCenter[1];
             qCenter_bar[2] = -qCenter[2];
             qCenter_bar[3] = -qCenter[3];

             QTtransform(qCenter_bar, temp1, zmp_local); // qCenter_bar*(zmp-pCenter)

             temp2[0] = GLOBAL_ZMP_REF_X;
             temp2[1] = GLOBAL_ZMP_REF_Y;
             temp2[2] = 0;
             diff_vv(temp2,3,pCenter,temp1);
             QTtransform(qCenter_bar, temp1, zmp_ref_local);


            X_ZMP_Local = 1000.*zmp_local[0];
            Y_ZMP_Local = 1000.*zmp_local[1];

            X_ZMP_Global = 1000.*zmp[0];
            Y_ZMP_Global = 1000.*zmp[1];

            X_ZMP_REF_Local = zmp_ref_local[0];
            Y_ZMP_REF_Local = zmp_ref_local[1];

            X_ZMP_REF_Global = GLOBAL_ZMP_REF_X;
            Y_ZMP_REF_Global = GLOBAL_ZMP_REF_Y;


//            X_ZMP_n =(1.f-2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f)*X_ZMP_n + 2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f*X_ZMP;
//            Y_ZMP_n =(1.f-2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f)*Y_ZMP_n + 2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f*Y_ZMP;
        }
}
void Kirk_Control()
{
    ZMP_intergral_control();
    final_gain_DSP_ZMP_CON = 0.5*(1-cos(PI*CNT_final_gain_DSP_ZMP_CON/40));

    Del_PC_X_DSP_XZMP_CON = final_gain_DSP_ZMP_CON*kirkZMPCon_XP2(-Del_PC_X_DSP_XZMP_CON, X_ZMP_Local - (X_ZMP_REF_Local)*1000 , 1);
    Del_PC_Y_DSP_YZMP_CON = final_gain_DSP_ZMP_CON*kirkZMPCon_YP2(-Del_PC_Y_DSP_YZMP_CON, Y_ZMP_Local - (Y_ZMP_REF_Local)*1000 , 1);

    LPF_Del_PC_X_DSP_XZMP_CON = (1.f-2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f)*LPF_Del_PC_X_DSP_XZMP_CON + 2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f*Del_PC_X_DSP_XZMP_CON;
    LPF_Del_PC_Y_DSP_YZMP_CON = (1.f-2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f)*LPF_Del_PC_Y_DSP_YZMP_CON + 2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f*Del_PC_Y_DSP_YZMP_CON;

    if(CNT_final_gain_DSP_ZMP_CON < 40) CNT_final_gain_DSP_ZMP_CON++;

    Old_Del_PC_X_DSP_XZMP_CON = Del_PC_X_DSP_XZMP_CON;
    Old_Del_PC_Y_DSP_YZMP_CON = Del_PC_Y_DSP_YZMP_CON;
    Old_Del_PC_X_DSP_XZMP_CON2 = Del_PC_X_DSP_XZMP_CON;
    Old_Del_PC_Y_DSP_YZMP_CON2 = Del_PC_Y_DSP_YZMP_CON;
    Old_I_ZMP_CON_X = I_ZMP_CON_X;
    Old_I_ZMP_CON_Y = I_ZMP_CON_Y;
}
void Kirk_Control_ssp()
{
    final_gain_SSP_ZMP_CON = 0.5*(1-cos(PI*CNT_final_gain_SSP_ZMP_CON/40));

    Del_PC_X_SSP_XZMP_CON = final_gain_SSP_ZMP_CON*kirkZMPCon_XP1(-Del_PC_X_SSP_XZMP_CON, X_ZMP_Local - (X_ZMP_REF_Local)*1000 , 1);
    Del_PC_Y_SSP_YZMP_CON = final_gain_SSP_ZMP_CON*kirkZMPCon_YP1(-Del_PC_Y_SSP_YZMP_CON, Y_ZMP_Local - (Y_ZMP_REF_Local)*1000 , 1);
    //printf(" Del_PC_Y_SSP_YZMP_CON = %f\n",Del_PC_Y_SSP_YZMP_CON);
    LPF_Del_PC_X_SSP_XZMP_CON = (1.f-2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f)*LPF_Del_PC_X_SSP_XZMP_CON + 2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f*Del_PC_X_SSP_XZMP_CON;
    LPF_Del_PC_Y_SSP_YZMP_CON = (1.f-2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f)*LPF_Del_PC_Y_SSP_YZMP_CON + 2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f*Del_PC_Y_SSP_YZMP_CON;

    if(CNT_final_gain_SSP_ZMP_CON < 40) CNT_final_gain_SSP_ZMP_CON++;

    Old_Del_PC_X_SSP_XZMP_CON = Del_PC_X_SSP_XZMP_CON;
    Old_Del_PC_Y_SSP_YZMP_CON = Del_PC_Y_SSP_YZMP_CON;
}
void ZMP_intergral_control()
{
    I_ZMP_CON_X += -0.001*(0.001*X_ZMP_Local - (X_ZMP_REF_Local ));//-X_ZMP_n_OFFSET_BP
    I_ZMP_CON_Y += -0.001*(0.001*Y_ZMP_Local - (Y_ZMP_REF_Local ));

    if(I_ZMP_CON_X > 0.04)I_ZMP_CON_X=0.04;
    else if(I_ZMP_CON_X < -0.04)I_ZMP_CON_X=-0.04;
    if(I_ZMP_CON_Y > 0.04)I_ZMP_CON_Y=0.04;
    else if(I_ZMP_CON_Y < -0.04)I_ZMP_CON_Y=-0.04;

}
double kirkZMPCon_YP1(double u, double ZMP, int zero)
{
    int i;
    const double A[2][2] = {{0,1},{-14.9504066667389	-0.616370808678501}};
    const double B[2] = {0,35.2914483893102};
    const double C[2] = {1.70719953272619,0.0382262996941896};
    const double D = -2.18871734964897;
//    const double Kg[2] = {1.87183004235299,0.492573413240444};
    const double Kg[2] = {0.596733608123605,0.322560555343192};
//    const double Og[2] = {10.2903310784662,47.5060572719318};
    const double Og[2] = {8.40760843975646,26.9490900208963};

    static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2];
    double y;

    Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
    Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

    Temp_2[0] = B[0]*u;
    Temp_2[1] = B[1]*u;

    Temp_3[0] = Og[0]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
    Temp_3[1] = Og[1]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

    if(zero == 0)
    {
        for(i=0; i<2; i++)
        {
            x_old[i] = 0.;
            x_new[i] = 0.;
            Temp_1[i] = 0.;
            Temp_2[i] = 0.;
            Temp_3[i] = 0.;
        }
    }

    x_new[0] = x_old[0] + DEL_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
    x_new[1] = x_old[1] + DEL_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);

    y = Kg[0]*x_new[0] + Kg[1]*x_new[1];

    x_old[0] = x_new[0];
    x_old[1] = x_new[1];

    if(y > 60.0) y = 60.0;
    else if(y < -60.0) y = -60.0;

    return y ;
}
// ----------------------------------------------------------------------------------- //
double kirkZMPCon_XP1(double u, double ZMP, int zero)
{
    int i;
    const double A[2][2] = {{0,1},{-42.8368246575059,-0.719099276791585}};
    const double B[2] = {0,71.0432663261910};
    const double C[2] = {3.43666912554807,0.0445973496432212};
    const double D = -4.40598605839496;
    const double Kg[2] = {0.537322920475309,	0.243244738267863};
    const double Og[2] = {5.40216194635022,	16.0426024573112};

    static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2];
    double y;

    Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
    Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

    Temp_2[0] = B[0]*u;
    Temp_2[1] = B[1]*u;

    Temp_3[0] = Og[0]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
    Temp_3[1] = Og[1]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

    if(zero == 0)
    {
        for(i=0; i<2; i++)
        {
            x_old[i] = 0.;
            x_new[i] = 0.;
            Temp_1[i] = 0.;
            Temp_2[i] = 0.;
            Temp_3[i] = 0.;
        }
    }

    x_new[0] = x_old[0] + DEL_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
    x_new[1] = x_old[1] + DEL_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);

    y = Kg[0]*x_new[0] + Kg[1]*x_new[1];

    x_old[0] = x_new[0];
    x_old[1] = x_new[1];

    if(y > 60.0) y = 60.0;
    else if(y < -60.0) y = -60.0;

    return y ;
}
double kirkZMPCon_XP2(double u, double ZMP, int zero)
{
    int i;
    const double A[2][2] = {{0.000000000000, 1.000000000000}, {-93.44,-1.0204}};
    const double B[2] = {0,153.5062};
    const double C[2] = {5.3672,0.0510};
    const double D = -7.6675;
    const double Kg[2] = {-0.1917,0.0976};
    const double Og[2] = {3.5220,1.4988};
//    const double A[2][2] = {{0.000000000000, 1.000000000000}, {-93.44,-0.8218}};
//    const double B[2] = {0,135.9192};
//    const double C[2] = {6.5750,0.0510};
//    const double D = -8.4295;
//    const double Kg[2] = {-0.2165,0.1117};
//    const double Og[2] = {2.9066,1.3227};
//    const double A[2][2] = {{0.000000000000, 1.000000000000}, {-121.509441687773,-0.686900801711364}};
//    const double B[2] = {0,171.905595852174};
//    const double C[2] = {8.31581491568207,0.0426004533905397};
//    const double D = -10.6613011739514;
//    const double Kg[2] = {-0.561351369682899,0.0541756604962248};
//    const double Og[2] = {1.87687421562180,-6.91634420265651};

    static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2];
    double y;

    Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
    Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

    Temp_2[0] = B[0]*u;
    Temp_2[1] = B[1]*u;

    Temp_3[0] = Og[0]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
    Temp_3[1] = Og[1]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

    if(zero == 0)
    {
        for(i=0; i<2; i++)
        {
            x_old[i] = 0.;
            x_new[i] = 0.;
            Temp_1[i] = 0.;
            Temp_2[i] = 0.;
            Temp_3[i] = 0.;
        }
    }

    x_new[0] = x_old[0] + DEL_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
    x_new[1] = x_old[1] + DEL_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);

    y = Kg[0]*x_new[0] + Kg[1]*x_new[1];

    x_old[0] = x_new[0];
    x_old[1] = x_new[1];

    if(y > 60.0) y = 60.0;
    else if(y < -60.0) y = -60.0;

    return y ;
}
// ----------------------------------------------------------------------------------- //
double kirkZMPCon_YP2(double u, double ZMP, int zero)
{
    int i;
//    const double A[2][2] = {{0.000000000000, 1.000000000000}, {-93.44,-1.2327}};
//    const double B[2] = {0,135.9192};
//    const double C[2] = {6.5750,0.0765};
//    const double D = -8.4295;
//    const double Kg[2] = {-0.0915,0.1234};
//    const double Og[2] = {2.8458,0.7336};
//    const double Kg[2] = {-0.4152,0.0792};
//

//    const double Og[2] = {2.8405,1.1907};
//

    const double A[2][2] = {{0.000000000000, 1.000000000000}, {-93.44,-1.5306}};
    const double B[2] = {0,153.5062};
    const double C[2] = {5.3672,0.0765};
    const double D = -7.6675;

    const double Kg[2] = {-0.0809742090339354,	0.107288107510564};
    const double Og[2] = {3.43082537995055,0.723663240519607};

//    const double A[2][2] = {{0.000000000000, 1.000000000000}, {-119.405421865731,-0.441579086814448}};
//    const double B[2] = {0,169.208134541865};
//    const double C[2] = {8.18532708084719,0.0273860057510612};
//    const double D = -10.4940090780092;
//    const double Kg[2] = {-0.552014961447503,0.0564891335695250};
//    const double Og[2] = {3.56807911031747,12.8724994839785};

    static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2];
    double y;

    Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
    Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

    Temp_2[0] = B[0]*u;
    Temp_2[1] = B[1]*u;

    Temp_3[0] = Og[0]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
    Temp_3[1] = Og[1]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

    if(zero == 0)
    {
        for(i=0; i<2; i++)
        {
            x_old[i] = 0.;
            x_new[i] = 0.;
            Temp_1[i] = 0.;
            Temp_2[i] = 0.;
            Temp_3[i] = 0.;
        }
    }

    x_new[0] = x_old[0] + DEL_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
    x_new[1] = x_old[1] + DEL_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);

   // y = Kg[0]*(atan2(GLOBAL_Y_LIPM_n,des_pCOM_3x1[Zdir])*1000-x_new[0]) + Kg[1]*(atan2(GLOBAL_Y_LIPM_d_n,des_pCOM_3x1[Zdir])*1000-x_new[1]);
    y = Kg[0]*(x_new[0]) + Kg[1]*(x_new[1]);

    x_old[0] = x_new[0];
    x_old[1] = x_new[1];

    if(y > 80.0) y = 80.0;
    else if(y < -80.0) y = -80.0;

    return y ;
}
