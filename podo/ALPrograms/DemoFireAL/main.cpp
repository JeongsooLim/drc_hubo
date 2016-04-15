// This AL is made by Inho Lee (HuboLab at KAIST)
// e-mail: inholee@kaist.ac.kr

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
#include "stdio.h"
#include "DebrisMotion.h"





CDebrisMotion m_dbmotion;




#define PODO_AL_NAME       "DemoFire_AL"

using namespace std;

// Functions ===========================================
// Signal handler
void CatchSignals(int _signal);
int HasAnyOwnership();

// Real-time thread for control
void *RBTaskThread(void *);
void *RBFlagThread(void *);

// Initialization
int RBInitialize();
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
JointControlClass *joint;
// =====================================================


// Command Set =========================================
enum DebrisALCOMMAND
{
	DemoFireAL_NO_ACT = 100,
	DemoFireAL_READYPOS,
	DemoFireAL_MOTIONSTART,
	DemoFireAL_TEST,
	DemoFireAL_SET,
	DemoFireAL_SW_READY,
	DemoFireAL_SW_SET,
	DemoFireAL_SW_GRASP,
	DemoFireAL_SW_HOLD,
	DemoFireAL_SW_BACK
};

int CheckMotionOwned();


// WBIK functins-------------------------------------------------------------------------------- //
// Variables
int             WB_FLAG = false;
long            LimitedJoint;
long            LimitType;
TaskMotion      *WBmotion;
// ---------------------------------------------------- //


void GotoReadyPos();
void GotoHomePos();
void GotoReadyPos_UB();

// ---------------------------------------------------- //

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
//		FILE_LOG(logERROR) << "No input argument";
//		//PODO_NO = 8;
//		return 0;
//	}else{
//		QString argStr;
//		argStr.sprintf("%s", argv[1]);
//		PODO_NO = argStr.toInt();
//		cout << endl << endl;
//		cout << "======================================================================" << endl;
//		cout << ">>> Process DemoFire_AL is activated..!!" << endl;
//		cout << ">>> PODO NAME: DemoFire_AL" << endl;
//		cout << ">>> PODO NO  : " << PODO_NO << endl;
//		cout << ">>> VERSION  : 2015.09.02." << PODO_NO << endl;
//		cout << "======================================================================" << endl;
//	}

	// Initialize RBCore -----------------------------------
	if(RBInitialize() == -1)
		isTerminated = -1;

    WBmotion = new TaskMotion(sharedREF, sharedSEN, sharedCMD, joint);
	//m_dbmotion.debMotion = new TaskMotion(sharedData, joint);


	m_dbmotion.Dock_Distance = 0.12;
	m_dbmotion.Wrist_Distance = 0.16;
	m_dbmotion.Palm_Distance = 0.035;
	m_dbmotion.Lift_Distance = 0.1;
	m_dbmotion.FT_Distance = -0.03;

	// User command cheking --------------------------------
	while(isTerminated == 0)
	{
		usleep(100*1000);

        switch(sharedCMD->COMMAND[PODO_NO].USER_COMMAND)
		{
			case DemoFireAL_SW_READY:
			{
				printf("## DemoFireAL_SW_READY  \n");


				joint->RefreshToCurrentReference();
				joint->SetAllMotionOwner();
				GotoReadyPos_UB();

			}
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = DemoFireAL_NO_ACT;
				break;

			case DemoFireAL_SW_SET:
			{
				printf("## DemoFireAL_SW_SET  \n");
                printf("Position    (m)   : %.3f %.3f %.3f \n", sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0], sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1], sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2]);
                printf("Orientation (deg) : %.1f \n", sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[3]);

                m_dbmotion.SW_Tp[0] = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                m_dbmotion.SW_Tp[1] = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                m_dbmotion.SW_Tp[2] = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];

				m_dbmotion.Temprv1[0] = 90*D2R;
				m_dbmotion.Temprv1[1] = 1;
				m_dbmotion.Temprv1[2] = 0;
				m_dbmotion.Temprv1[3] = 0;
				m_dbmotion.RV2QT(m_dbmotion.Temprv1, m_dbmotion.Tempth1);

                m_dbmotion.Temprv2[0] = (-90 + sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[3])*D2R;
				m_dbmotion.Temprv2[1] = 0;
				m_dbmotion.Temprv2[2] = 1;
				m_dbmotion.Temprv2[3] = 0;
				m_dbmotion.RV2QT(m_dbmotion.Temprv2, m_dbmotion.Tempth2);

				m_dbmotion.QTCross(m_dbmotion.Tempth1, m_dbmotion.Tempth2, m_dbmotion.Desth);

				m_dbmotion.SW_Tw[0] = m_dbmotion.Desth[0];
				m_dbmotion.SW_Tw[1] = m_dbmotion.Desth[1];
				m_dbmotion.SW_Tw[2] = m_dbmotion.Desth[2];
				m_dbmotion.SW_Tw[3] = m_dbmotion.Desth[3];

			}
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = DemoFireAL_NO_ACT;
				break;

			case DemoFireAL_SW_GRASP:
			{
				printf("## DemoFireAL_SW_GRASP  \n");

				m_dbmotion.WBIKframe(WBmotion, WBmode_RGLG);

				m_dbmotion.MotionType = MTYPE_SW_GRASP;
				m_dbmotion.StartMotion();

			}
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = DemoFireAL_NO_ACT;
				break;

			case DemoFireAL_SW_HOLD:
			{
				printf("## DemoFireAL_SW_HOLD  \n");

				m_dbmotion.WBIKframe(WBmotion, WBmode_RGLG);

				m_dbmotion.MotionType = MTYPE_SW_HOLD;
				m_dbmotion.StartMotion();

			}
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = DemoFireAL_NO_ACT;
				break;

			case DemoFireAL_SW_BACK:
			{
				printf("## DemoFireAL_SW_BACK  \n");

				m_dbmotion.WBIKframe(WBmotion, WBmode_RGLG);

				m_dbmotion.MotionType = MTYPE_SW_BACK;
				m_dbmotion.StartMotion();

			}
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = DemoFireAL_NO_ACT;
				break;




        case DemoFireAL_SET:
        {
            printf("## DemoFireAL_SET %d \n", sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]);

            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 0)
                m_dbmotion.bEncCheck = false;
            else
                m_dbmotion.bEncCheck = true;
        }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = DemoFireAL_NO_ACT;
            break;
		case DemoFireAL_READYPOS:
		{
            printf("## DemoFireAL_READYPOS %d a \n", sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]);
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 1){

				joint->RefreshToCurrentReference();
				joint->SetAllMotionOwner();

				GotoReadyPos();
			}
			else{

				joint->RefreshToCurrentReference();
				joint->SetAllMotionOwner();

				GotoHomePos();
			}


            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = DemoFireAL_NO_ACT;
			break;
		}

		case DemoFireAL_MOTIONSTART:
		{
            printf("## DemoFireAL_MOTIONSTART %d \n", sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]);


            double tableheight = 0.72;

			// #1
            m_dbmotion.DebG[0].Pos[0] = 0.47;
            m_dbmotion.DebG[0].Pos[1] = -0.15;
            m_dbmotion.DebG[0].Pos[2] = tableheight + 0.18;

            m_dbmotion.DebG[0].Ori[0] = -90;
            m_dbmotion.DebG[0].Ori[1] = 20;

			// #2
            m_dbmotion.DebG[1].Pos[0] = 0.47;
            m_dbmotion.DebG[1].Pos[1] = 0.2;
            m_dbmotion.DebG[1].Pos[2] = tableheight + 0.18;

            m_dbmotion.DebG[1].Ori[0] = 45;
            m_dbmotion.DebG[1].Ori[1] = 0;

			// #3
            m_dbmotion.DebG[2].Pos[0] = 0.6;
            m_dbmotion.DebG[2].Pos[1] = -0.15;
            m_dbmotion.DebG[2].Pos[2] = tableheight + 0.18;

            m_dbmotion.DebG[2].Ori[0] = 0;
			m_dbmotion.DebG[2].Ori[1] = 0;

			// #4
            m_dbmotion.DebG[3].Pos[0] = 0.47;
            m_dbmotion.DebG[3].Pos[1] = 0.2;
            m_dbmotion.DebG[3].Pos[2] = tableheight + 0.18;

            m_dbmotion.DebG[3].Ori[0] = -45;
			m_dbmotion.DebG[3].Ori[1] = 0;




			m_dbmotion.WBIKframe(WBmotion, WBmode_RGLG);
			m_dbmotion.DebG[0] = m_dbmotion.TransFormDebInfo2Global(m_dbmotion.DebG[0]);
			m_dbmotion.DebG[1] = m_dbmotion.TransFormDebInfo2Global(m_dbmotion.DebG[1]);
			m_dbmotion.DebG[2] = m_dbmotion.TransFormDebInfo2Global(m_dbmotion.DebG[2]);
			m_dbmotion.DebG[3] = m_dbmotion.TransFormDebInfo2Global(m_dbmotion.DebG[3]);

            // #1
            m_dbmotion.DebS[0].Pos[0] = 0.55;
            m_dbmotion.DebS[0].Pos[1] = -0.125;
            m_dbmotion.DebS[0].Pos[2] = 0.81;

            m_dbmotion.DebS[0].Ori[0] = -90;
            m_dbmotion.DebS[0].Ori[1] = 0;

            // #2
            m_dbmotion.DebS[1].Pos[0] = 0.55;
            m_dbmotion.DebS[1].Pos[1] = 0.125;
            m_dbmotion.DebS[1].Pos[2] = 0.81;

            m_dbmotion.DebS[1].Ori[0] = 90;
            m_dbmotion.DebS[1].Ori[1] = 0;

            // #3
            m_dbmotion.DebS[2].Pos[0] = 0.55;
            m_dbmotion.DebS[2].Pos[1] = -0.33;
            m_dbmotion.DebS[2].Pos[2] = 0.81;

            m_dbmotion.DebS[2].Ori[0] = -90;
            m_dbmotion.DebS[2].Ori[1] = 0;

            // #4
            m_dbmotion.DebS[3].Pos[0] = 0.55;
            m_dbmotion.DebS[3].Pos[1] = 0.33;
            m_dbmotion.DebS[3].Pos[2] = 0.81;

            m_dbmotion.DebS[3].Ori[0] = 90;
            m_dbmotion.DebS[3].Ori[1] = 0;



            m_dbmotion.DebS[0] = m_dbmotion.TransFormDebInfo2Global(m_dbmotion.DebS[0]);
            m_dbmotion.DebS[1] = m_dbmotion.TransFormDebInfo2Global(m_dbmotion.DebS[1]);
            m_dbmotion.DebS[2] = m_dbmotion.TransFormDebInfo2Global(m_dbmotion.DebS[2]);
            m_dbmotion.DebS[3] = m_dbmotion.TransFormDebInfo2Global(m_dbmotion.DebS[3]);


//            printf("cur Pelz %.3f %.3f\n", WBmotion->pPelZ, WBmotion->des_pPELz);

			// move debris right side to left side
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 1){
				//m_dbmotion.MotionType = MTYPE_TEST;
				m_dbmotion.MotionType = MTYPE_PICKRIGHTSIDE;
				m_dbmotion.StartMotion();
			}
			// re-arrange debris left to right side
			else{
                m_dbmotion.MotionType = MTYPE_STACKLEFTSIDE;
                m_dbmotion.StartMotion();
			}


            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = DemoFireAL_NO_ACT;
			break;
		}

		case DemoFireAL_TEST:
		{
			printf("## DemoFireAL_TEST \n");
\

            printf("Rfing enc print %.3f \n", sharedSEN->ENCODER[MC_GetID(RHAND)][MC_GetCH(RHAND)].CurrentPosition);

			m_dbmotion.bEncOK = true;
            m_dbmotion.bEncOKRH = true;
            m_dbmotion.bEncOKLH = true;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = DemoFireAL_NO_ACT;
			break;
		}


		default:
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = DemoFireAL_NO_ACT;
			break;
		}

	}
	FILE_LOG(logERROR) << "Process DemoFireAL is terminated" << endl;
	return 0;







	cout << "Hello World!" << endl;
	return 0;
}





void *RBTaskThread(void *)
{
	while(isTerminated == 0)
	{
        checkSuspend();

		// update cnt of the motion script
		if(m_dbmotion.bMotionGenerator == true)
		{
			m_dbmotion.MotionGenerator();
			if(1){
				if(m_dbmotion.bRHFTon == true)
					m_dbmotion.RHFTFeedBack();
				if(m_dbmotion.bLHFTon == true)
                    m_dbmotion.LHFTFeedBack();
                if(m_dbmotion.bEncCheck == true){
                    if(m_dbmotion.bEncCheckRH == true)
                        m_dbmotion.EncCheckRFin();
                    if(m_dbmotion.bEncCheckLH == true)
                        m_dbmotion.EncCheckLFin();
                }


			}
		}


		// update the joint values and push it to the shared memory
		if(WB_FLAG == true && WBmotion->IK_status == 0){
			WBmotion->updateAll();
			WBmotion->WBIK_UB();
			//WBmotion->WBIK();

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


			joint->SetJointRefAngle(WST, WBmotion->Q_filt_34x1[idWST]*R2D);

			joint->SetJointRefAngle(RHY, WBmotion->Q_filt_34x1[idRHY]*R2D);
			joint->SetJointRefAngle(RHR, WBmotion->Q_filt_34x1[idRHR]*R2D);
			joint->SetJointRefAngle(RHP, WBmotion->Q_filt_34x1[idRHP]*R2D);
			joint->SetJointRefAngle(RKN, WBmotion->Q_filt_34x1[idRKN]*R2D);
			joint->SetJointRefAngle(RAP, WBmotion->Q_filt_34x1[idRAP]*R2D);
			joint->SetJointRefAngle(RAR, WBmotion->Q_filt_34x1[idRAR]*R2D);

			joint->SetJointRefAngle(LHY, WBmotion->Q_filt_34x1[idLHY]*R2D);
			joint->SetJointRefAngle(LHR, WBmotion->Q_filt_34x1[idLHR]*R2D);
			joint->SetJointRefAngle(LHP, WBmotion->Q_filt_34x1[idLHP]*R2D);
			joint->SetJointRefAngle(LKN, WBmotion->Q_filt_34x1[idLKN]*R2D);
			joint->SetJointRefAngle(LAP, WBmotion->Q_filt_34x1[idLAP]*R2D);
			joint->SetJointRefAngle(LAR, WBmotion->Q_filt_34x1[idLAR]*R2D);

			if(!CheckMotionOwned())
				WB_FLAG = false;
		}





		// update the internal joint values and suspend this thread
		joint->MoveAllJoint();
		joint->SetReferenceReady();
        suspendThread();
	}
}

void *RBFlagThread(void *)
{
	while(isTerminated == 0)
	{
        usleep(300);
		if(HasAnyOwnership()){
            if(sharedCMD->SYNC_SIGNAL[PODO_NO] == true){
				if(joint->CheckReferenceReady() == false){
					FILE_LOG(logWARNING) << "AL thread takes too long time";
				}
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
        if(sharedCMD->MotionOwner[MC_GetID(i)][MC_GetCH(i)] == PODO_NO)
			return true;
	}
	return false;
}
// --------------------------------------------------------------------------------------------- //

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

    sharedCMD->COMMAND[PODO_NO].USER_COMMAND = DemoFireAL_NO_ACT;
	return 0;
}
// --------------------------------------------------------------------------------------------- //
int CheckMotionOwned()
{
	for(int i=0;i<(NO_OF_JOINTS-2);i++)
	{
        if(sharedCMD->MotionOwner[MC_GetID(i)][MC_GetCH(i)] !=PODO_NO)	return 0;
	}
	return 1;
}

void GotoReadyPos_UB()
{
	WB_FLAG = false;
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

void GotoReadyPos()
{
	WB_FLAG = false;
	double postime = 4000.0;

	joint->SetMoveJoint(RSP, 20.0, postime, MOVE_ABSOLUTE);
	joint->SetMoveJoint(RSR, 10.0, postime, MOVE_ABSOLUTE);
	joint->SetMoveJoint(RSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -130, postime, MOVE_ABSOLUTE);
	joint->SetMoveJoint(RWY, 0.0, postime, MOVE_ABSOLUTE);
	joint->SetMoveJoint(RWP, 30.0, postime, MOVE_ABSOLUTE);

	joint->SetMoveJoint(LSP, 20.0, postime, MOVE_ABSOLUTE);
	joint->SetMoveJoint(LSR, -10.0, postime, MOVE_ABSOLUTE);
	joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -130.0, postime, MOVE_ABSOLUTE);
	joint->SetMoveJoint(LWY, 0.0, postime, MOVE_ABSOLUTE);
	joint->SetMoveJoint(LWP, 30.0, postime, MOVE_ABSOLUTE);

	joint->SetMoveJoint(WST, 0.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RWY2, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RHAND, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY2, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LHAND, 0.0, postime, MOVE_ABSOLUTE);

	joint->SetMoveJoint(RHY, 0, postime, MOVE_ABSOLUTE);
	joint->SetMoveJoint(RHR, 0, postime, MOVE_ABSOLUTE);
	joint->SetMoveJoint(RHP, -40.66, postime, MOVE_ABSOLUTE);
	joint->SetMoveJoint(RKN, 75.02, postime, MOVE_ABSOLUTE);
	joint->SetMoveJoint(RAP, -34.35, postime, MOVE_ABSOLUTE);
	joint->SetMoveJoint(RAR, 0, postime, MOVE_ABSOLUTE);

	joint->SetMoveJoint(LHY, 0, postime, MOVE_ABSOLUTE);
	joint->SetMoveJoint(LHR, 0, postime, MOVE_ABSOLUTE);
	joint->SetMoveJoint(LHP, -40.66, postime, MOVE_ABSOLUTE);
	joint->SetMoveJoint(LKN, 75.02, postime, MOVE_ABSOLUTE);
	joint->SetMoveJoint(LAP, -34.35, postime, MOVE_ABSOLUTE);
	joint->SetMoveJoint(LAR, 0, postime, MOVE_ABSOLUTE);


}

void GotoHomePos()
{
	WB_FLAG = false;
	double postime = 3000.0;
	for(int i=0; i<NO_OF_JOINTS; i++)
		joint->SetMoveJoint(i, 0.0, postime, MOVE_ABSOLUTE);
}


