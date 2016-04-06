#ifndef MANUALCAN_H
#define MANUALCAN_H
#include "../../SHARE/Headers/RBSharedMemory.h"
#include "../../SHARE/Headers/JointInformation.h"

// Manual CAN for GainOverride
#define SW_MODE_COMPLEMENTARY           0x00
#define SW_MODE_NON_COMPLEMENTARY       0x01

const struct {
    int canch;          // CAN Channel
    int bno;            // Board Number
    int mch;            // Motor Channel
} JOINT_INFO[NO_OF_JOINTS] = {
    {0,0,1}, {0,1,1}, {0,2,1}, {0,3,1}, {0,4,1}, {0,5,1},       // RHY, RHR, RHP, RKN, RAP, RAR,
    {1,6,2}, {1,7,2}, {1,8,2}, {1,9,2}, {1,10,2},{1,11,2},      // LHY, LHR, LHP, LKN, LAP, LAR,
    {2,13,1}, {2,14,1}, {2,15,1}, {2,15,2}, {2,16,1}, {2,16,2}, // RSP, RSR, RSY, REB, RWY, RWP,
    {3,17,1}, {3,18,1}, {3,19,1}, {3,19,2}, {3,20,1}, {3,20,2}, // LSP, LSR, LSY, LEB, LWY, LWP,
    {0,12,1},                                                   // WST,
    {2,36,1},{2,36,2},{3,37,1},{3,37,2},                        // RF1, RF2, LF1, LF2,
    {0,4,2}, {1,10,2}                                           // RWH, LWH
};


int	PushCANMessage(MANUAL_CAN MCData);
int MCJointGainOverride(unsigned int _canch, unsigned int _bno, int _mch, int logscale, short _msec);
int MCBoardSetSwitchingMode(unsigned int _canch, unsigned int _bno, int _mode);
int MCenableFrictionCompensation(unsigned int _canch, unsigned int _bno, unsigned int _mch, unsigned int _enable);
int MCsetFrictionParameter(unsigned int _canch, unsigned int _bno,
                                    unsigned int _mch, short _vel_saturation, int _amp_compen, int _vel_dead);
int MCJointEnableFeedbackControl(unsigned int _canch, unsigned int _bno, unsigned int _mch, int _enable);
int MCJointSetControlMode(unsigned int _canch, unsigned int _bno, unsigned int _mch, int _mode);
int MCJointPWMCommand2chHR(unsigned int _canch, unsigned int _bno, int mode1, short duty1, int mode2, short duty2);
int MCWristFTsensorNull(unsigned int _canch, unsigned int _bno);
int MCJointRequestEncoderPosition(unsigned int _canch, unsigned _bno, int _mode);


//////////////OKKEE ADDED



int MCJointFindHome(unsigned int _canch, unsigned int _bno, int _mch);




//////////////FINDHOME
#endif // MANUALCAN_H
