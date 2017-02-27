#include "ManualCAN.h"

//extern pRBCORE_SHM sharedData;

// --------------------------------------------------------------------------------------------- //
int PushCANMessage(MANUAL_CAN MCData)
{
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
//            return 0;
//        }
//    }
//    cout << "Fail to send Manual CAN..!!" << endl; //fir simulation
    return 1;
}

// --------------------------------------------------------------------------------------------- //
int MCJointGainOverride(unsigned int _canch, unsigned int _bno, int _mch, int logscale, short _msec)
{
    // MsgID        Byte0       Byte1       Byte2             	Byte3		Byte4
    // 0x21+BNO		command     MCH(1~2)    LOG_SCALE(1~100)	DURATION	DURATION
    MANUAL_CAN	MCData;

    MCData.channel = _canch;
    MCData.id = 0x210 + _bno;
    MCData.dlc = 5;
    MCData.data[0] = 0x6F;                  // command
    MCData.data[1] = _mch;                  // motor channel
    MCData.data[2] = logscale;              // 1~100 -- 0.9^logscale
    MCData.data[3] = (unsigned char)(_msec & 0x00FF);
    MCData.data[4] = (unsigned char)((_msec>>8) & 0x00FF);

    return PushCANMessage(MCData);
}

// --------------------------------------------------------------------------------------------- //
int MCBoardSetSwitchingMode(unsigned int _canch, unsigned int _bno, int _mode)
{
    // MsgID        Byte0       Byte1
    // 0x210+BNO	command     _mode (0x00 : complementary, 0x01 : non complementary)
    MANUAL_CAN	MCData;

    MCData.channel = _canch;
    MCData.id = 0x210 + _bno;
    MCData.dlc = 2;
    MCData.data[0] = 0x13;         // command
    MCData.data[1] = _mode;        // (0x00 : complementary, 0x01 : non complementary)

    return PushCANMessage(MCData);
}

// --------------------------------------------------------------------------------------------- //
int MCenableFrictionCompensation(unsigned int _canch, unsigned int _bno, unsigned int _mch, unsigned int _enable)
{
    // MsgID		Byte0   Byte1                 Byte2
    // 0x210+BNO	command    MCH(0:all, 1~2:MCH)   _enable (1:ENABLE, 0:DISABLE)
    MANUAL_CAN	MCData;

    MCData.channel = _canch;
    MCData.id = 0x210 + _bno;
    MCData.dlc = 3;
    MCData.data[0] = 0xB1;      // command
    MCData.data[1] = _mch;      // motor channel
    MCData.data[2] = _enable;   // 1-enable, 0-disable

    return PushCANMessage(MCData);
}

// --------------------------------------------------------------------------------------------- //
int MCsetFrictionParameter(unsigned int _canch, unsigned int _bno,
                                    unsigned int _mch, short _vel_saturation, int _amp_compen, int _vel_dead) // inhyeok
{
    // MsgID		Byte0       Byte1       Byte2	Byte3		Byte4   	Byte5
    // 0x210+BNO	command     MCH(1~2)	VEL		VEL         AMP(mA)		VEL_DEAD

    //         compensating current(mA)
    //                         |
    //               (AMP) --> |     _____
    //                         |    /
    //                         |   / ^
    //                      ______/  |______(VEL)
    //         -------------------------------------joint velocity(pulse/0.01sec)
    //                    /    |  ^
    //                   /     |  |______(VEL_DEAD)
    //            ______/      |
    //                         |

    MANUAL_CAN	MCData;

    MCData.channel = _canch;
    MCData.id = 0x210 + _bno;
    MCData.dlc = 6;
    MCData.data[0] = 0xB0;							// command
    MCData.data[1] = _mch;							// motor channel
    MCData.data[2] = (unsigned char)(_vel_saturation & 0x00FF);
    MCData.data[3] = (unsigned char)((_vel_saturation>>8) & 0x00FF); // pulse/0.01sec
    MCData.data[4] = _amp_compen;                   // mA
    MCData.data[5] = _vel_dead;                     // pulse/msec

    return PushCANMessage(MCData);
}

// --------------------------------------------------------------------------------------------- //
int MCJointEnableFeedbackControl(unsigned int _canch, unsigned int _bno, unsigned int _mch, int _enable)
{
    // MsgID            Byte0       Byte1                  Byte2
    // 0x210+BNO		command		MCH(0:all, 1~2:MCH)    _enable (1:ENABLE, 0:DISABLE)

    MANUAL_CAN	MCData;

    MCData.channel = _canch;
    MCData.id = 0x210 + _bno;
    MCData.dlc = 3;
    MCData.data[0] = 0x0E;       // command
    MCData.data[1] = _mch;       // motor channel
    MCData.data[2] = _enable;    // 1-enable, 0-disable


    return PushCANMessage(MCData);
}

int MCJointSetControlMode(unsigned int _canch, unsigned int _bno, unsigned int _mch, int _mode){
    // MsgID            Byte0       Byte1                  Byte2
    // 0x210+BNO		command		MCH(0:all, 1~2:MCH)    _mode (1:Current, 0:Position)

    MANUAL_CAN	MCData;

    MCData.channel = _canch;
    MCData.id = 0x210 + _bno;
    MCData.dlc = 3;
    MCData.data[0] = 0x10;       // command
    MCData.data[1] = _mch;       // motor channel
    MCData.data[2] = _mode;    // 1-Current, 0-Position

    return PushCANMessage(MCData);
}

// --------------------------------------------------------------------------------------------- //
int MCJointPWMCommand2chHR(unsigned int _canch, unsigned int _bno, int mode1, short duty1, int mode2, short duty2)
{
    // MsgID		Byte0       Byte1       Byte2	Byte3                   Byte4       Byte5	Byte6
    // 0x210+BNO	command		mode_ch1	duty1	duty1,current1(mode 4)	mode_ch2	duty2	duty2,current2(mode 4)

    // modeX ================
    // 0: not applied for channel X
    // 1: open-loop PWM in % duty
    // 2: open-loop PWM in 0.1% resolution duty
    // 3: feed-forward open-loop PWM in 0.1% resolution duty
    // 4: feed-forward current-mapped PWM(mA)                    ---> OL current command mode

    MANUAL_CAN	MCData;

    MCData.channel = _canch;
    MCData.id = 0x210 + _bno;
    MCData.dlc = 7;
    MCData.data[0] = 0x0D;           // command
    MCData.data[1] = mode1;
    MCData.data[2] = (duty1 & 0x00FF);
    MCData.data[3] = ((duty1>>8) & 0x00FF);
    MCData.data[4] = mode2;
    MCData.data[5] = (duty2 & 0x00FF);
    MCData.data[6] = ((duty2>>8) & 0x00FF);

    return PushCANMessage(MCData);
}

int MCWristFTsensorNull(unsigned int _canch, unsigned int _bno){
    // hand -> 0= Right 1=left 2=both
    // MsgID                Byte0	Byte1   Byte2
    // CANID_TXDF   		BNO		0x81    _mode

    MANUAL_CAN	MCData;

    MCData.channel = _canch;
    MCData.id = COMMAND_CANID;
    MCData.dlc = 3;

    MCData.data[0] = _bno;		// board no.
    MCData.data[1] = 0x81;		// command
    MCData.data[2] = 0x00;		// mode
    //mode = 0x00 : FT sensor
    //mode = 0x04 : Inclinometers in FT sensor

    return PushCANMessage(MCData);
}

// New Protocol -- 2015/02/16 -- jeongsoo, inhyeok
int MCJointRequestEncoderPosition(unsigned int _canch, unsigned _bno, int _mode)
{
    // MsgID            Byte0       Byte1                  Byte2
    // 0x210+BNO		command		MCH(0:all, 1~2:MCH)    _mode (1:Continuous, 0:one-time)

    MANUAL_CAN	MCData;
    MCData.channel = _canch;
    MCData.id = 0x210 + _bno;
    MCData.dlc = 2;
    MCData.data[0] = 0x03;
    MCData.data[1] = _mode;      // 1-continuous, 0-oneshot



    return PushCANMessage(MCData);
}



int MCJointFindHome(unsigned int _canch, unsigned int _bno, int _mch)
{
    MANUAL_CAN MCData;
    MCData.id =  0x210 + _bno;
    MCData.channel = _canch;
    MCData.data[0] = 0x11;
    MCData.data[1] = _mch; //1,2,0
    MCData.dlc = 2;


    return PushCANMessage(MCData);


}
