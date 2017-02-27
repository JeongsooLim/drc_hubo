#ifndef OMNIWHEELVARIABLES_H
#define OMNIWHEELVARIABLES_H

// Baseic const
#ifndef PI
#define PI			3.141592653589793
#endif
#ifndef D2R
#define D2R			1.745329251994330e-2
#endif
#ifndef R2D
#define R2D			5.729577951308232e1
#endif

// Local Command
#define IDLE_STATUS                 0
#define WHEEL_OPERATION             3
#define WHEEL_VELMODE               6
#define WHEEL_MANUALMODE            9
#define WHEEL_RADIUSMODE            12
#define WHEEL_ROSMODE               50

// Motion number
#define MOTION_NONE                 100
#define MOTION_1                    0
#define MOTION_2                    1
#define MOTION_3                    2
#define COMPEN_1                    3
#define COMPEN_3                    4
#define COMPEN_1_1                  5
#define COMPEN_3_1                  6

#define RADIUS_1                    1
#define RADIUS_2                    2
#define RADIUS_3                    3
#define RADIUS_4                    4


// STATUS CHECK
#define STATUS_IDLE                 1
#define STATUS_OPERATE              0
enum OMNIWHEEL_ALCOMMAND
{
    OMNIWHEEL_AL_NO_ACT = 100,
    OMNIWHEEL_AL_GOTODES,
    OMNIWHEEL_AL_VELMODE,
    OMNIWHEEL_AL_CHANGEPOS,
    OMNIWHEEL_AL_CONTROL,
    OMNIWHEEL_AL_MANUAL,
    OMNIWHEEL_AL_RADIUS,
    OMNIWHEEL_AL_ROS
};
enum WALKREADYCOMMAND
{
    WALKREADY_NO_ACT = 100,
    WALKREADY_GO_WALKREADYPOS,
    WALKREADY_GO_HOMEPOS,
    WALKREADY_WHEELCMD
};

// Hip Pitch Gain Over
unsigned int _GAIN_HIP_FLAG = false;
const double _GAIN_HIP_PITCH = 40;

// Wheel mode angle
const double _POS_HIP_PITCH = -53.35;//-52.95;//-53.25;//
const double _POS_ANKLE_PITCH = -7.95;//-8.15;//

const double SET_MAX_WHEEL = 350.;

// ROBOT 1
//const int _GAIN_RWH = 110;
//const int _GAIN_LWH = 162;
// ROBOT 2
//const int _GAIN_RWH = 125;
//const int _GAIN_LWH = 142;

const int _GAIN_RWH = 110;
const int _GAIN_LWH = 142;


// STATE MACHINE
char STATUS_FLAG=STATUS_IDLE;
// General Constant
const double _robot_width=0.378;//m 0.39 in silnae
const double _wheel_radi=0.059;//m 0.0588
double _max_wheel_rot= 350.;//deg/s
const double _max_wheel_acc_time=1;//s

unsigned int velChangeFlag = false;
unsigned int noWaitFlag = false;
unsigned int directCompenFlag = false;

unsigned int radius_velChangeFlag = false;

// Flag
char _EM_Stop_Flag=0;
unsigned long _localCount;
char _localCommand;// selection in Thread
int _localMotionNum;// selection in localCommand
char _isFirst=1;

// Input Variables
double _des_x=0.;
double _des_y=0.;
double _des_a=0.;

// General val
double maxLinearVel=0.2;//m/s
double maxAngularVel=90.;//deg/s

double maxLinearAcc=0.2;//m/s^2
double maxAngularAcc=90.;//deg/s^2

// Planned Result
double _path_plan[3][2];

// Path Calculator
unsigned int OmniPathGenerator(const double _des_x, const double _des_y, const double _des_a, char _mode);

// Angle regulator
double OW_angle_regul(const double input_angle);

// Thread
int inicator=0;
int inicator_radius=0;
int OW_Operation_in_RealTime(unsigned long ulCount, int motion_number);
int OW_Operation_in_RealTime_Radius(unsigned long ulCount, int motion_number);
int OW_WHEEL_MOVEMENT(void);
// Tankmode
int Tank_Mode_Flag = false;
double Tank_delta =0.;

// Final output
double _OW_RIGHT=0.;//deg/s
double _OW_LEFT=0.;//deg/s

// OW_Operation variables
double cur_Distance;
double cur_T1, cur_T2, cur_T3;
unsigned long cur_T1_count, cur_T2_count, cur_T3_count;
unsigned long cur_rest_count=5;//count
unsigned long gap_end_T1, gap_end_T2, gap_end_T3;
double _GOAL_DELTA=0.;

// Task mode
void StartWBIKmotion(int _mode);
double _PEL_Z_WALKREADY = 0.745139;
double _FOOT_CENT_X;
double _FOOT_CENT_X2;
double _COM_X_WALKREADY;

// Manual mode
int L_JOG_RL=0;
int AROW_RL=0;
int R_JOG_UD=0;

int L_JOG_RL_pre=0;
int AROW_RL_pre=0;
int R_JOG_UD_pre=0;

static unsigned long JOG_SAME_COUNT=0;

double RWHnow, LWHnow;
void CalculateMovingEverage(void);
void WheelMoveManual(void);

// Radius mode
int WMR_dir=1;
int WMR_fb=0;
double WMR_radius;
double WMR_angle=70;

double WMR_omega_global_max;
double WMR_alpha_global_max;
// Save File
unsigned int saveFlag=0;
unsigned int saveIndex=0;
float DataBuf[10][100000];
void SaveFile(void);
unsigned int debugFlag=0;
void InitWheelHome(void);

// Change Pos Function
unsigned long pos_change_count=0;
char WALKtoWHEEL_FLAG=false;
char WHEELtoWALK_FLAG=false;
void WALKtoWHEEL_FUNCTION(void);
void WHEELtoWALK_FUNCTION(void);

void WALKtoWHEEL_FUNCTION_FAST(void);
void WHEELtoWALK_FUNCTION_FAST(void);

// Yaw Compensation

unsigned int yawFlag = 0;
double _OW_YAW_VALUE = 0.;
double _OW_YAW_DES = 0.;
double _OW_DIRCT = 0.;

double _compen_1[2];
double _compen_1_1[2];
double _compen_3[2];
double _compen_3_1[2];

double _r_compen_1[2];
double _r_compen_2[2];

// Controller
unsigned char Control_ON = true;
double _OW_CONTROL_RWH =0.;
double _OW_CONTROL_LWH =0.;

// Pos changer
double delta_pos_angle = 16.;

// Knee gain function
void Knee_Gain_Over(void);
void Knee_Gain_Return(void);

unsigned char REAL_MODE = true;

void Wheel_Walk_Ready(double pos_ms);
#endif // OMNIWHEELVARIABLES_H
