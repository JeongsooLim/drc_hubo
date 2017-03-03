#ifndef JOINT_H
#define JOINT_H

#include <iostream>
#include <math.h>

#include "../../SHARE/Headers/JointInformation.h"
#include "../../SHARE/Headers/RBSharedMemory.h"
#include "../../SHARE/Headers/UserSharedMemory.h"
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
    int             MCId;
    int             MCCh;

	double			RefAngleCurrent;
	double			RefAngleDelta;
	double			RefAngleToGo;
	double			RefAngleInitial;
    unsigned long	GoalTimeCount;
	unsigned long	CurrentTimeCount;
	unsigned char	MoveFlag;

public:
	JointClass()							{RefAngleCurrent = 0.f; MoveFlag = DISABLE;}
	JointClass(const int id, const int ch)	{MCId = id; MCCh = ch; RefAngleCurrent = 0.f; MoveFlag = DISABLE;}

	int				GetId()									{return MCId;}
	int				GetCh()									{return MCCh;}
	void			SetRefAngleCurrent(const double ref)	{RefAngleCurrent = ref;}
	double			GetRefAngleCurrent()					{return RefAngleCurrent;}
    void            SetMoveFlag(unsigned char flag)         {MoveFlag = flag;}

	unsigned char	SetMoveJoint(const double _angle, const double _msTime, const unsigned int _mode);
	unsigned char	MoveJoint();
};

// JointControlClass with JointClass Vector
class JointControlClass
{
private:


    int				PODONum;
    pRBCORE_SHM_REFERENCE   Shm_ref;
    pRBCORE_SHM_SENSOR      Shm_sen;
    pRBCORE_SHM_COMMAND     Shm_com;
public:

    JointVector		Joints;
    explicit JointControlClass(pRBCORE_SHM_REFERENCE _shm_ref, pRBCORE_SHM_SENSOR _shm_sen, pRBCORE_SHM_COMMAND _shm_com, int _podoNum);

    void			RefreshToCurrentReference();

    int				GetJointNumber(const int _mcNum, const int _mcCh);
    JointClass*		GetJointClass(const int n)							{return Joints[n];}
    double			GetJointRefAngle(const int n)						{return Joints[n]->GetRefAngleCurrent();}
    void			SetJointRefAngle(const int n, const double _ref)	{Joints[n]->SetRefAngleCurrent(_ref);}

    void			SetMotionOwner(const int _jointNum);
    void			SetAllMotionOwner();

    unsigned char	SetMoveJoint(const int _jointNum, const double _angle, const double _msTime, const unsigned int _mode);
    unsigned char	MoveJoint(const int _jointNum);
    void            MoveAllJoint();
    void			JointUpdate();
};

#endif // JOINT_H

