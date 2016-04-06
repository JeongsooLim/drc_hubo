

#include "BasicFiles/BasicSetting.h"


// Basic --------
pRBCORE_SHM_COMMAND     sharedCMD;
pRBCORE_SHM_REFERENCE   sharedREF;
pRBCORE_SHM_SENSOR      sharedSEN;
pUSER_SHM               userData;
JointControlClass       *jCon;


int     __IS_WORKING = false;
int     __IS_GAZEBO = false;
int     PODO_NO = -1;


int main(int argc, char *argv[])
{
    // Termination signal ---------------------------------
    signal(SIGTERM, CatchSignals);   // "kill" from shell
    signal(SIGINT,  CatchSignals);    // Ctrl-c
    signal(SIGHUP,  CatchSignals);    // shell termination
    signal(SIGKILL, CatchSignals);

    // Block memory swapping ------------------------------
    mlockall(MCL_CURRENT|MCL_FUTURE);

    // Setting the AL Name <-- Must be a unique name!!
    sprintf(__AL_NAME, "ALTutorial");


    CheckArguments(argc, argv);
    if(PODO_NO == -1){
        FILE_LOG(logERROR) << "Please check the AL Number";
        FILE_LOG(logERROR) << "Terminate this AL..";
        return 0;
    }

    // Initialize RBCore -----------------------------------
    if(RBInitialize() == false)
        __IS_WORKING = false;


    while(__IS_WORKING){
        usleep(100*1000);

        switch(sharedCMD->COMMAND[PODO_NO].USER_COMMAND){
        case 999:
            FILE_LOG(logSUCCESS) << "Command 999 received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();
            jCon->SetMoveJoint(WST, 30.0, 2000.0, MOVE_ABSOLUTE);
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        default:
            break;
        }
    }

    FILE_LOG(logERROR) << "Process \"" << __AL_NAME << "\" is terminated" << endl;
    return 0;
}



//==============================//
// Task Thread
//==============================//
void *RBTaskThread(void *)
{
    while(__IS_WORKING)
    {
        checkSuspend();

        jCon->MoveAllJoint();
        suspendThread();
    }
}
//==============================//



//==============================//
// Flag Thread
//==============================//
void *RBFlagThread(void *)
{
    while(__IS_WORKING)
    {
        usleep(300);

        if(HasAnyOwnership()){
            if(sharedCMD->SYNC_SIGNAL[PODO_NO] == true){
                jCon->JointUpdate();
                resumeThread();
            }
        }
    }
}
//==============================//
