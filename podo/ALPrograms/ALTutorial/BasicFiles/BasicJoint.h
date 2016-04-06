#ifndef BASICJOINT_H
#define BASICJOINT_H

#include <iostream>
#include <math.h>

#include "RBSharedMemory.h"
#include "JointInformation.h"
#include "RBLog.h"

class JointClass;
typedef QVector<JointClass*> JointVector;

// enum variables
enum ErrCode{
    ERR_OK = 0,
    ERR_GOAL_TIME,
    ERR_ALREADY_MOVING,
    ERR_WRONG_MODE,
    ERR_WRONG_SELECTION
};
enum MovingStatus{
    MOVE_DONE = 0,
    STILL_MOVING
};
enum MoveCommandMode{
    MOVE_RELATIVE = 0,
    MOVE_ABSOLUTE
};


class JointClass
{
public:
    int         JNum;
    int         MCId;
    int         MCCh;
    double      RefAngleCurrent;

public:
    JointClass(){
        RefAngleCurrent = 0.f; MoveFlag = false;
    }
    JointClass(const int jnum, const int id, const int ch){
        JNum = jnum; MCId = id; MCCh = ch; RefAngleCurrent = 0.f; MoveFlag = false;
    }

    void    SetRefAngleCurrent(const double ref)	{RefAngleCurrent = ref;}
    double  GetRefAngleCurrent()					{return RefAngleCurrent;}
    void    SetMoveFlag(unsigned char flag)         {MoveFlag = flag;}

    char    SetMoveJoint(const double _angle, const double _msTime, const unsigned int _mode){
        if(_msTime <= 0){
            FILE_LOG(logWARNING) << "Goal time must be greater than zero(SetMoveJoint)[JNum: " << JNum << "]"; return ERR_GOAL_TIME;
        }

        MoveFlag = false;
        switch(_mode)
        {
        case MOVE_RELATIVE:	// relative mode
            RefAngleToGo = RefAngleCurrent + _angle;
            break;
        case MOVE_ABSOLUTE:	// absolute mode
            RefAngleToGo = _angle;
            break;
        default:
            FILE_LOG(logWARNING) << "Wrong reference mode(SetMoveJoint)[JNum: " << JNum << "]";
            return ERR_WRONG_MODE;
            break;
        }
        RefAngleInitial = RefAngleCurrent;
        RefAngleDelta = RefAngleToGo - RefAngleCurrent;
        CurrentTimeCount = 0;

        GoalTimeCount = (unsigned long)(_msTime/RT_TIMER_PERIOD_MS);
        MoveFlag = true;
        return ERR_OK;
    }
    char    MoveJoint(){
        if(MoveFlag == true){
            CurrentTimeCount++;
            if(GoalTimeCount <= CurrentTimeCount){
                GoalTimeCount = CurrentTimeCount = 0;
                RefAngleCurrent = RefAngleToGo;
                MoveFlag = false;
                return MOVE_DONE;
            }else{
                RefAngleCurrent = RefAngleInitial+RefAngleDelta*0.5f*(1.0f-cos(RBCORE_PI/(double)GoalTimeCount*(double)CurrentTimeCount));
            }
        }
        return STILL_MOVING;
    }

private:
    double			RefAngleDelta;
    double			RefAngleToGo;
    double			RefAngleInitial;
    unsigned long	GoalTimeCount;
    unsigned long	CurrentTimeCount;
    unsigned char	MoveFlag;
};



class JointControlClass
{
public:

    explicit JointControlClass(pRBCORE_SHM_REFERENCE _shm_ref, pRBCORE_SHM_SENSOR _shm_sen, pRBCORE_SHM_COMMAND _shm_com, int _podoNum){
        Shm_ref = _shm_ref;
        Shm_sen = _shm_sen;
        Shm_com = _shm_com;
        PODONum = _podoNum;
        Joints = JointVector(NO_OF_JOINTS);
        for(int i=0; i<NO_OF_JOINTS; i++)
            Joints[i] = new JointClass(i, MC_GetID(i), MC_GetCH(i));
    }

    JointVector		Joints;

    double  GetJointRefAngle(const int n)						{return Joints[n]->GetRefAngleCurrent();}
    void    SetJointRefAngle(const int n, const double _ref)	{Joints[n]->SetRefAngleCurrent(_ref);}

    void    SetMotionOwner(const int _jnum){
        Shm_com->MotionOwner[Joints[_jnum]->MCId][Joints[_jnum]->MCCh] = PODONum;
    }
    void    SetAllMotionOwner(){
        for(int i=0; i<NO_OF_JOINTS; i++){
            SetMotionOwner(i);
        }
    }

    char	SetMoveJoint(const int _jnum, const double _angle, const double _msTime, const unsigned int _mode){
        return Joints[_jnum]->SetMoveJoint(_angle, _msTime, _mode);
    }
    char	MoveJoint(const int _jnum){
        return Joints[_jnum]->MoveJoint();
    }
    void    MoveAllJoint(){
        for(int i=0; i<NO_OF_JOINTS; i++){
            MoveJoint(i);
        }
    }

    void    JointUpdate(){
        int mcId, mcCh;
        for(int i=0; i<NO_OF_JOINTS; i++){
            mcId = Joints[i]->MCId;
            mcCh = Joints[i]->MCCh;
            Shm_ref->JointReference[PODONum][mcId][mcCh] = Joints[i]->GetRefAngleCurrent();
            if(Shm_com->MotionOwner[mcId][mcCh] == PODONum)
                Shm_com->ACK_SIGNAL[mcId][mcCh] = true;
        }
        Shm_com->SYNC_SIGNAL[PODONum] = false;
    }
    void    RefreshToCurrentReference(){
        int mcId, mcCh;
        for(int i=0; i<NO_OF_JOINTS; i++){
            Joints[i]->SetMoveFlag(false);
            mcId = Joints[i]->MCId;
            mcCh = Joints[i]->MCCh;
            Joints[i]->SetRefAngleCurrent(Shm_sen->ENCODER[mcId][mcCh].CurrentReference);
            Shm_ref->JointReference[PODONum][mcId][mcCh] = Joints[i]->GetRefAngleCurrent();
        }
    }

private:
    int                     PODONum;
    pRBCORE_SHM_REFERENCE   Shm_ref;
    pRBCORE_SHM_SENSOR      Shm_sen;
    pRBCORE_SHM_COMMAND     Shm_com;
};

#endif // BASICJOINT_H
