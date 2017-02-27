#ifndef HUBO2REQUEST_H
#define HUBO2REQUEST_H

//#include "../Share/include/Hubo2Def.h"
//#include "../Share/include/Hubo2Command.h"
#include <isnl/math/geometry.h>
#include <string.h>
#include <stdio.h>

const char JOINT_NAME[35][5] = {
	"RHY", "RHR", "RHP", "RKN", "RAP", "RAR",
	"LHY", "LHR", "LHP", "LKN", "LAP", "LAR",
	"RSP", "RSR", "RSY", "REB", "RWY", "RWP",
	"LSP", "LSR", "LSY", "LEB", "LWY", "LWP",
	"NKY", "NK1", "NK2", "WST",
	"RF1", "RF2", "RF3", "LF1", "LF2", "LF3",
	"NULL"
};
const char TASK_SPACE_NAME[10][15] = {
	"RFoot", "LFoot", "RHand", "LHand",
	"Pelvis", "CoM", "WST",
	"RStick", "LStick", "NULL"
};
const char TASK_COORD_NAME[4][15] = {
	"Global", "Local", "Upper", "NULL"
};
const char TASK_MODE_NAME[4][15] = {
	"Absolute", "Relative", "End Rel", "NULL"
};
const char TASK_ACTION_NAME[3][15] = {
	"Execute", "Regist", "NULL"
};
const char TASK_INTERP_NAME[4][15] = {
	"Spline", "Cosine", "Linear", "NULL"
};


const int TASK_SPACE    =0x000000ff;
const int TASK_JOINT    =0x000000ff;
const int TASK_COORD    =0x00000f00; // >> 8
const int TASK_MODE     =0x0000f000; // >> 12
const int TASK_ACTION   =0x000f0000; // >> 16
const int TASK_INTERP   =0x00f00000; // >> 20

const int TASK_GLOBAL   =0x00000000; // Task space on global coordinate
const int TASK_LOCAL    =0x00000100; // Task space on local coordinate (Pelvis based)
const int TASK_UPPER    =0x00000200; // Task space on local coordinate (Upper body based)
const int NO_OF_TASK_COORD = 3;

const int TASK_MODEABS  =0x00000000; // Absolute control mode
const int TASK_MODEREL  =0x00001000; // Relative control mode
const int TASK_MODEEND  =0x00002000; // Relative control mode based on end effector
const int NO_OF_TASK_MODE = 3;

const int TASK_EXECUTE  =0x00000000; // instantly execute
const int TASK_REGIST   =0x00010000; // register via point
const int NO_OF_TASK_ACTION = 2;

const int TASK_FORCEDIR =0x00000000; // Force mode : give directional force
const int TASK_FORCEREF =0x00010000; // Force mode : give reference position

const int TASK_SPLINE   =0x00000000; // use spline interpolation
const int TASK_COSINE   =0x00100000; // use 1-cos interpolation
const int TASK_LINEAR   =0x00200000; // use linear interpolation : use for aleady interpolated samples
const int NO_OF_TASK_INTERP = 3;
inline int _imin(int i, int j){return (i < j ? i : j);}

enum SimpleCommand{
	// Pre-defined motion
	POS_HOME, POS_WALK_READY, POS_TASK_READY,
	// Motion Register handling command
	MF_ACCEPT_REG, MF_CLEAR_REG, MF_PERFORM,
	// Control mode
	CTRL_NONE, CTRL_JOINT, CTRL_TASK,

	N_SCOMM // number of simple command
};
enum Hubo2Params{
	// int parameters : Flags...
	IK_FIX_RW, IK_FIX_LW, IK_LSTICK, IK_RSTICK,

	// int readonly parameters
	CONTROL_SPACE,   // JointSpaceFlag : 0 = TaskSpace, 1 = JointSpace

	// float parameters
	IK_RELB_ANG, IK_LELB_ANG,
	IK_KPRED, IK_KDRED,

	IK_MASS_PEL,
	IK_MASS_TOR,
	IK_MASS_ULEG,
	IK_MASS_LLEG,
	IK_MASS_FOOT,
	IK_MASS_UARM,
	IK_MASS_LARM,
	IK_MASS_LHND,
	IK_MASS_RHND,


	N_PARAM
};
const char SIMPLE_COMM_NAME[N_SCOMM][20] = {
	"Home Pose", "Walk-ready Pose", "Task-ready Pose",
	"Register Accept", "Register Clear", "Register Perform",
	"No Control", "Joint Space Control", "Task Space Control",
};

#define MAX_FRAME 3
//struct RequestTaskSPMotion{
//public:
//	int id;    // command ID : should be MANUAL_MOTION (in Hubo2Command.h)
//	int tid;   // TASK_SPACE | TASK_MODE | TASK_COORD | TASK_ACTION | TASK_INTERP
//	float duration;
//	isnl::pos p;
//	RequestTaskSPMotion():id(MANUAL_MOTION){}
//	RequestTaskSPMotion(int tid, const isnl::pos& p, float duration = 3.f):id(MANUAL_MOTION){
//		this->tid      = tid;
//		this->duration = duration;
//		this->p        = p;
//	}
//	RequestTaskSPMotion(const HUBO2_LAN_COMMAND& comm):id(MANUAL_MOTION){
//		decode(comm);
//	}
//	void encode(HUBO2_LAN_COMMAND& comm) const {
//		comm.Command = id;
//		comm.CommandIntData[0] = tid;
//		comm.CommandFloatData[0] = duration;
//		memcpy(comm.CommandFloatData+1,p.v,sizeof(isnl::pos));
//	}
//	void decode(const HUBO2_LAN_COMMAND& comm){
//		id    = comm.Command;
//		tid   = comm.CommandIntData[0];
//		duration = comm.CommandFloatData[0];
//		memcpy(p.v,comm.CommandFloatData+1,sizeof(isnl::pos));
//	}
//	void print() const {


//		int tsid   = _imin( tid & TASK_SPACE, NO_OF_TASK_SPACE);
//		int coord  = _imin((tid & TASK_COORD)>>8, NO_OF_TASK_COORD);
//		int mode   = _imin((tid & TASK_MODE)>>12, NO_OF_TASK_MODE);
//		int action = _imin((tid & TASK_ACTION)>>16, NO_OF_TASK_ACTION);
//		int interp = _imin((tid & TASK_INTERP)>>20, NO_OF_TASK_INTERP);

//		const char *str_task    = TASK_SPACE_NAME[tsid];
//		const char *str_coord   = TASK_COORD_NAME[coord];
//		const char *str_mode    = TASK_MODE_NAME[mode];
//		const char *str_action  = TASK_ACTION_NAME[action];
//		const char *str_interp  = TASK_INTERP_NAME[interp];

//		printf("Request Taskspace Motion : %s %s %s %s %s\n", str_action, str_task, str_mode, str_coord, str_interp);
//		printf("[%5.2f] %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f\n",
//			   duration, p[0],p[1],p[2],p[3],p[4],p[5],p[6]);
//	}
//};
//struct RequestJointMotion{
//	int id;    // command ID : should be MANUAL_JOINT (in Hubo2Command.h)
//	int njoint;// number of point, number of joint
//	int jid[NO_OF_JOINTS]; // joint ids
//	float duration;        // motion duration
//	float data[NO_OF_JOINTS]; // end point, via point, via point... [start point (no need to send)]
//	RequestJointMotion():id(MANUAL_JOINT){}
//	RequestJointMotion(int id, float value, float duration=3.f):id(MANUAL_JOINT){
//		this->njoint  = 1;
//		this->duration = duration;
//		this->jid[0]  = id;
//		this->data[0] = value;
//	}
//	RequestJointMotion(const ints& id, const floats& value, float duration=3.f):id(MANUAL_JOINT){
//		this->njoint  = id.size();
//		this->duration = duration;
//		for(int i = 0; i < njoint; ++i){
//			this->jid[i]  = id[i];
//			this->data[i] = value[i];
//		}
//	}
//	RequestJointMotion(const HUBO2_LAN_COMMAND& comm):id(MANUAL_JOINT){
//		decode(comm);
//	}
//	void encode(HUBO2_LAN_COMMAND& comm) const {
//		comm.Command = id;
//		comm.CommandIntData[0] = njoint;
//		memcpy(comm.CommandIntData+3,jid, njoint*sizeof(int));
//		comm.CommandFloatData[0] = duration;
//		memcpy(comm.CommandFloatData+1,data,njoint*sizeof(float));
//	}
//	void decode(const HUBO2_LAN_COMMAND& comm){
//		id    = comm.Command;
//		njoint= comm.CommandIntData[0];
//		memcpy(jid, comm.CommandIntData+3,njoint*sizeof(int));
//		duration = comm.CommandFloatData[0];
//		memcpy(data,comm.CommandFloatData+1,njoint*sizeof(float));
//	}
//	void print() const {
//		int mode   = _imin((jid[0] & TASK_MODE)>>12, NO_OF_TASK_MODE);
//		int action = _imin((jid[0] & TASK_ACTION)>>16, NO_OF_TASK_ACTION);
//		int interp = _imin((jid[0] & TASK_INTERP)>>20, NO_OF_TASK_INTERP);

//		const char *str_mode    = TASK_MODE_NAME[mode];
//		const char *str_action  = TASK_ACTION_NAME[action];
//		const char *str_interp  = TASK_INTERP_NAME[interp];

//		printf("%x, %d, %d, %d, %d\n", jid[0], action, mode, interp, njoint);
//		printf("%x, %x, %x, %x\n", jid[0], TASK_EXECUTE, TASK_MODEREL, TASK_LINEAR);
//		printf("Request Joint Motion : %s %s %s\n", str_action, str_mode, str_interp);
//		printf("[%2d]   ",njoint);
//		for(int i = 0; i < njoint; ++i)
//			printf("   %s ",JOINT_NAME[jid[i] & TASK_JOINT]);
//		printf("\n");
//		printf("[%5.2f]", duration);
//		for(int i = 0; i < njoint; ++i)
//			printf("%6.1f ",data[i]);
//		printf("\n");
//	}
//};
//struct RequestSimpleCommand{
//	int id;    // command ID : should be MANUAL_SIMPLE_COMMAND (in Hubo2Command.h)
//	int command;
//	RequestSimpleCommand():id(MANUAL_SIMPLE_COMMAND){}
//	RequestSimpleCommand(int comm):id(MANUAL_SIMPLE_COMMAND),command(comm){}
//	RequestSimpleCommand(const HUBO2_LAN_COMMAND& comm):id(MANUAL_SIMPLE_COMMAND){
//		decode(comm);
//	}
//	void encode(HUBO2_LAN_COMMAND& comm) const {
//		comm.Command = id;
//		comm.CommandIntData[0] = command;
//	}
//	void decode(const HUBO2_LAN_COMMAND& comm){
//		id      = comm.Command;
//		command = comm.CommandIntData[0];
//	}
//	void print() const {
//		printf("Simple Command : %s\n", SIMPLE_COMM_NAME[command]);
//	}
//};
//struct RequestFParamSet{
//	int id;    // command ID : should be MANUAL_FPARAM_SET (in Hubo2Command.h)
//	int pid; // parameter id
//	float value;
//	RequestFParamSet():id(MANUAL_FPARAM_SET){}
//	RequestFParamSet(int pid, float value):id(MANUAL_FPARAM_SET),pid(pid),value(value){}
//	RequestFParamSet(const HUBO2_LAN_COMMAND& comm):id(MANUAL_FPARAM_SET){
//		decode(comm);
//	}
//	void encode(HUBO2_LAN_COMMAND& comm) const {
//		comm.Command             = id;
//		comm.CommandIntData[0]   = pid;
//		comm.CommandFloatData[0] = value;
//	}
//	void decode(const HUBO2_LAN_COMMAND& comm){
//		id    = comm.Command;
//		pid   = comm.CommandIntData[0];
//		value = comm.CommandFloatData[0];
//	}
//	void print() const {
//		printf("Set parameter %d with value %f\n", pid, value);
//	}
//};
//struct RequestIParamSet{
//	int id;    // command ID : should be MANUAL_IPARAM_SET (in Hubo2Command.h)
//	int pid; // parameter id
//	int value;
//	RequestIParamSet():id(MANUAL_IPARAM_SET){}
//	RequestIParamSet(int pid, int value):id(MANUAL_IPARAM_SET),pid(pid),value(value){}
//	RequestIParamSet(const HUBO2_LAN_COMMAND& comm):id(MANUAL_IPARAM_SET){
//		decode(comm);
//	}
//	void encode(HUBO2_LAN_COMMAND& comm) const {
//		comm.Command             = id;
//		comm.CommandIntData[0]   = pid;
//		comm.CommandFloatData[0] = value;
//	}
//	void decode(const HUBO2_LAN_COMMAND& comm){
//		id    = comm.Command;
//		pid   = comm.CommandIntData[0];
//		value = comm.CommandFloatData[0];
//	}
//	void print() const {
//		printf("Set parameter %d with value %d\n", pid, value);
//	}
//};
//struct RequestGainOverride{
//	int id;    // command ID : should be COMP_GAINOVERRIDE (in Hubo2Command.h)
//	int time;
//	int value[15];
//	RequestGainOverride():id(COMP_GAINOVERRIDE){
//		time = 10; //
//		for(int i = 0; i < 15; ++i)
//			value[i] = 1000;
//	}
//	RequestGainOverride(const HUBO2_LAN_COMMAND& comm):id(COMP_GAINOVERRIDE){
//		decode(comm);
//	}
//	void setRArm(int val){
//		for(int i = 0; i < 7; ++i)
//			value[i] = val;
//	}
//	void setLArm(int val){
//		for(int i = 8; i < 14; ++i)
//			value[i] = val;
//	}
//	void setWST(int val){
//		value[14] = val;
//	}
//	void encode(HUBO2_LAN_COMMAND& comm) const {
//		comm.Command             = id;
//		comm.CommandIntData[0]   = time;
//		memcpy(comm.CommandIntData+1, value, 15*sizeof(int));
//	}
//	void decode(const HUBO2_LAN_COMMAND& comm){
//		id    = comm.Command;
//		time  = comm.CommandIntData[0];
//		memcpy(value, comm.CommandIntData+1, 15*sizeof(int));
//	}
//	void print() const {
//		printf("Command Gain override : time : %d\n", time);
//		printf("RArm : %4d %4d %4d %4d %4d %4d %4d\n",            value[0], value[1], value[2], value[3],  value[4],  value[5],  value[6]);
//		printf("LArm : %4d %4d %4d %4d %4d %4d %4d, WST : %4d\n", value[7], value[8], value[9], value[10], value[11], value[12], value[13], value[14]);
//	}
//};
//struct RequestTaskSPForce{
//	int id;
//	int mode;         // TASK_MODE | TASK_COORD | TASK_ACTION(FORCE)
//	int rflag, lflag; // 0 = disable, 1 = enable, 2 = leave current setting
//	isnl::pos rp, lp;

//	RequestTaskSPForce():id(MANUAL_TASK_FORCE){
//		rflag = 2; lflag = 2;
//	}
//	RequestTaskSPForce(int flag):id(MANUAL_TASK_FORCE){
//		rflag = flag; lflag = flag;
//	}
//	RequestTaskSPForce(int tid, int flag):id(MANUAL_TASK_FORCE){
//		if((tid & TASK_SPACE)==TASK_RH){
//			rflag = flag;
//			lflag = 2;
//		}
//		if((tid & TASK_SPACE)==TASK_LH){
//			lflag = flag;
//			rflag = 2;
//		}
//	}
//	RequestTaskSPForce(int tid, const isnl::pos& p):id(MANUAL_TASK_FORCE){
//		mode = tid;
//		if((tid & TASK_SPACE)==TASK_RH){
//			rflag = 1;
//			lflag = 2;
//			rp = p;
//		}
//		if((tid & TASK_SPACE)==TASK_LH){
//			lflag = 1;
//			rflag = 2;
//			lp = p;
//		}
//	}
//	RequestTaskSPForce(int mode, const isnl::pos& rp, const isnl::pos& lp):id(MANUAL_TASK_FORCE){
//		rflag = 1; lflag = 1;
//		this->mode = mode;
//		this->rp = rp;
//		this->lp = lp;
//	}
//	RequestTaskSPForce(const HUBO2_LAN_COMMAND& comm):id(MANUAL_TASK_FORCE){
//		decode(comm);
//	}
//	void encode(HUBO2_LAN_COMMAND& comm) const {
//		comm.Command             = id;
//		comm.CommandIntData[0]   = mode;
//		comm.CommandIntData[1]   = rflag;
//		comm.CommandIntData[2]   = lflag;
//		memcpy(comm.CommandFloatData,   rp.v, sizeof(isnl::pos));
//		memcpy(comm.CommandFloatData+7, lp.v, sizeof(isnl::pos));
//	}
//	void decode(const HUBO2_LAN_COMMAND& comm){
//		id    = comm.Command;
//		mode  = comm.CommandIntData[0];
//		rflag = comm.CommandIntData[1];
//		lflag = comm.CommandIntData[2];
//		memcpy(rp.v, comm.CommandFloatData,   sizeof(isnl::pos));
//		memcpy(lp.v, comm.CommandFloatData+7, sizeof(isnl::pos));
//	}
//	void print() const {
//		char str_mode[15]  = "NULL";
//		char str_coord[15] = "NULL";

//		switch(mode & TASK_MODE){
//		case TASK_MODEABS: strcpy(str_mode, "Absolute"); break;
//		case TASK_MODEREL: strcpy(str_mode, "Relative"); break;
//		case TASK_MODEEND: strcpy(str_mode, "Hand Rel"); break;
//		}

//		switch(mode & TASK_COORD){
//		case TASK_GLOBAL: strcpy(str_coord, "Global"); break;
//		case TASK_LOCAL:  strcpy(str_coord, "Local"); break;
//		case TASK_UPPER:  strcpy(str_coord, "Upper"); break;
//		}

//		int action = mode & TASK_ACTION;

//		printf("Setting force : %s, %s, (%d/%d)\n", str_mode, str_coord, rflag, lflag);
//		if(rflag==0) printf("RH Force Disable\n");
//		if(lflag==0) printf("LH Force Disable\n");
//		if(rflag==1 && action == TASK_FORCEDIR)
//			printf("RH Dir : %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f\n", rp.x, rp.y, rp.z, rp.qx, rp.qy, rp.qz);
//		if(lflag==1 && action == TASK_FORCEDIR)
//			printf("LH Dir : %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f\n", rp.x, rp.y, rp.z, rp.qx, rp.qy, rp.qz);
//		if(rflag==1 && action == TASK_FORCEREF)
//			printf("RH Ref  : %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f\n", rp.x, rp.y, rp.z, rp.qw, rp.qx, rp.qy, rp.qz);
//		if(lflag==1 && action == TASK_FORCEREF)
//			printf("LH Ref  : %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f\n", lp.x, lp.y, lp.z, lp.qw, lp.qx, lp.qy, lp.qz);
//	}
//};
struct RequestBipedWalk{
public:
	int id;    // command ID : should be MOTION_KIRK_BIPED (in Hubo2Command.h)
	int step;
	float Ls;    // meter
	float Lss;   // meter
	float rot;   // degree
	float Hs;    // meter
	float Ab;    // meter
	float delay; // ratio
	RequestBipedWalk():id(MOTION_KIRK_BIPED){
		this->step  = 0;
		this->Ls    =  0.00f;
		this->Lss   =  0.00f;
		this->rot   =  0.00f;
		this->Hs    =  0.35f;
		this->Ab    =  0.48f;
		this->delay =  0.01f;
	}
	RequestBipedWalk(int step, float Ls, float Lss, float rot, float Hs, float Ab, float delay):id(MOTION_KIRK_BIPED){
		this->step  = step;
		this->Ls    = Ls;
		this->Lss   = Lss;
		this->rot   = rot;
		this->Hs    = Hs;
		this->Ab    = Ab;
		this->delay = delay;
	}
	RequestBipedWalk(const HUBO2_LAN_COMMAND& comm):id(MOTION_KIRK_BIPED){
		decode(comm);
	}
	void encode(HUBO2_LAN_COMMAND& comm) const {
		comm.Command             = id;
		comm.CommandCharData[0]  = (step==0 ? 3 : 4);
		comm.CommandIntData[0]   = step;
		comm.CommandFloatData[0] =    Ls*100.f;
		comm.CommandFloatData[1] =   Lss*100.f;
		comm.CommandFloatData[2] =   rot;
		comm.CommandFloatData[3] =    Hs*100.f;
		comm.CommandFloatData[4] = delay;
		comm.CommandFloatData[5] =    Ab*100.f;
	}
	void decode(const HUBO2_LAN_COMMAND& comm){
		id    = comm.Command;
		step  = comm.CommandIntData[0];
		Ls    = comm.CommandFloatData[0]/100.f;
		Lss   = comm.CommandFloatData[1]/100.f;
		rot   = comm.CommandFloatData[2];
		Hs    = comm.CommandFloatData[3]/100.f;
		delay = comm.CommandFloatData[4];
		Ab    = comm.CommandFloatData[5]/100.f;
	}
	void print() const {
		if(step==0)
			printf("Walk Stop\n");
		else
			printf("Walk %d steps : %3.0fmm,%3.0fmm,%3.0fdeg,%3.0fmm,%3.0fmm,%5.3f\n",step,Ls,Lss,rot,Hs,Ab,delay);

	}
};
#endif




