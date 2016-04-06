#include "joint.h"


unsigned char JointClass::SetMoveJoint(const double _angle, const double _msTime, const unsigned int _mode){
	if(_msTime <= 0) { std::cout << ">>> Goal time must be grater than zero..!!(setMoveJointAngle)" << std::endl; return ERR_GOAL_TIME; }

	MoveFlag = DISABLE;
	switch(_mode)
	{
	case MOVE_RELATIVE:	// relative mode
		RefAngleToGo = RefAngleCurrent + _angle;
		break;
	case MOVE_ABSOLUTE:	// absolute mode
		RefAngleToGo = _angle;
		break;
	default:
		std::cout << ">>> Wrong reference mode(RBsetMoveJointAngle)..!!" << std::endl;
		return ERR_WRONG_MODE;
		break;
	}
	RefAngleInitial = RefAngleCurrent;
	RefAngleDelta = RefAngleToGo - RefAngleCurrent;
	CurrentTimeCount = 0;

	GoalTimeCount = (unsigned long)(_msTime/RT_TIMER_PERIOD_MS);
	MoveFlag = ENABLE;
	return ERR_OK;
}

unsigned char JointClass::MoveJoint(){
	// for reference generator
	if(MoveFlag == ENABLE){
		CurrentTimeCount++;
		if(GoalTimeCount <= CurrentTimeCount){
			GoalTimeCount = CurrentTimeCount = 0;
			RefAngleCurrent = RefAngleToGo;
			MoveFlag = DISABLE;
			return MOVE_DONE;
		}else{
			RefAngleCurrent = RefAngleInitial+RefAngleDelta*0.5f*(1.0f-cos(RBCORE_PI/(double)GoalTimeCount*(double)CurrentTimeCount));
		}
	}
	return STILL_MOVING;
}


JointControlClass::JointControlClass(pRBCORE_SHM_REFERENCE _shm_ref, pRBCORE_SHM_SENSOR _shm_sen, pRBCORE_SHM_COMMAND _shm_com, int _podoNum){
    Shm_ref = _shm_ref;
    Shm_sen = _shm_sen;
    Shm_com = _shm_com;
    PODONum = _podoNum;
	Joints = JointVector(NO_OF_JOINTS);
	for(int i=0; i<NO_OF_JOINTS; i++)
		Joints[i] = new JointClass(MC_ID_CH_Pairs[i].id, MC_ID_CH_Pairs[i].ch);
}

void JointControlClass::RefreshToCurrentReference(){
    int mcId, mcCh;
    for(int i=0; i<NO_OF_JOINTS; i++){
        Joints[i]->SetMoveFlag(DISABLE);
        mcId = Joints[i]->GetId();
        mcCh = Joints[i]->GetCh();
        Joints[i]->SetRefAngleCurrent(Shm_sen->ENCODER[mcId][mcCh].CurrentReference);
        Shm_ref->JointReference[PODONum][mcId][mcCh] = Joints[i]->GetRefAngleCurrent();
    }
}

void JointControlClass::SetMotionOwner(const int _jointNum){
	int mcId = Joints[_jointNum]->GetId();
	int mcCh = Joints[_jointNum]->GetCh();
    Shm_com->MotionOwner[mcId][mcCh] = PODONum;
}

void JointControlClass::SetAllMotionOwner(){
	for(int i=0; i<NO_OF_JOINTS; i++)
		SetMotionOwner(i);
}

unsigned char JointControlClass::SetMoveJoint(const int _jointNum, const double _angle, const double _msTime, const unsigned int _mode){
	return Joints[_jointNum]->SetMoveJoint(_angle, _msTime, _mode);
}
unsigned char JointControlClass::MoveJoint(const int _jointNum){
	return Joints[_jointNum]->MoveJoint();
}
void JointControlClass::MoveAllJoint(){
    int mcId, mcCh;
    for(int i=0; i<NO_OF_JOINTS; i++){
        mcId = Joints[i]->GetId();
        mcCh = Joints[i]->GetCh();
        MoveJoint(i);
    }
}

void JointControlClass::JointUpdate(){
    int mcId, mcCh;
    for(int i=0; i<NO_OF_JOINTS; i++){
        mcId = Joints[i]->GetId();
        mcCh = Joints[i]->GetCh();
        Shm_ref->JointReference[PODONum][mcId][mcCh] = Joints[i]->GetRefAngleCurrent();
        if(Shm_com->MotionOwner[mcId][mcCh] == PODONum)
            Shm_com->ACK_SIGNAL[mcId][mcCh] = true;
    }
    Shm_com->SYNC_SIGNAL[PODONum] = false;
}

int JointControlClass::GetJointNumber(const int _mcId, const int _mcCh){
	int res = -1;
	for(int i=0; i<NO_OF_JOINTS; i++){
		if(Joints[i]->GetId() == _mcId && Joints[i]->GetCh() == _mcCh)
			return i;
	}
	return res;
}

