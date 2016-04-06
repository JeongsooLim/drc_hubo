#ifndef RBDATATYPE_H
#define RBDATATYPE_H

#include <QString>
#include "../SHARE/Headers/RBSharedMemory.h"


typedef	unsigned int	uint;
typedef unsigned char	uchar;
typedef	unsigned long	ulong;

extern pRBCORE_SHM_COMMAND      sharedCMD;
extern pRBCORE_SHM_REFERENCE    sharedREF;
extern pRBCORE_SHM_SENSOR       sharedSEN;


//======================================================
// Database data type for General
typedef struct _DB_GENERAL_{
    int     VERSION;
    int     NO_OF_AL;
    int     NO_OF_COMM_CH;      // communication channel
    int     NO_OF_MC;           // motor controller
    int     NO_OF_FT;           // ft sensor
    int     NO_OF_IMU;          // imu sensor
    int     NO_OF_SP;           // smart power controller
    int     NO_OF_OF;           // optic flow sensor
}DB_GENERAL;

// Database data type for Motor Controller
typedef struct _DB_JOINT_{
    double  HARMONIC;
    double  PULLY_DRIVE;
    double  PULLY_DRIVEN;
    double  ENCODER_RESOLUTION;
    double  PPR;
    double  FRIC_PARAM1;
    double  FRIC_PARAM2;
}DB_JOINT;

typedef struct _DB_MC_{
    int     BOARD_ID;
    int     BOARD_TYPE;
    int     MOTOR_CHANNEL;
    int     CAN_CHANNEL;
    int     ID_SEND_REF;
    int     ID_RCV_ENC;
    int     ID_RCV_STAT;
    int     ID_RCV_INFO;
    int     ID_RCV_PARA;
    int     ID_SEND_GENERAL;
    DB_JOINT    JOINTS[MAX_JOINT];
}DB_MC;

// Database data type for FT Sensor
typedef struct _DB_FT_{
    int     BOARD_ID;
    int     SENSOR_ID;
    int     CAN_CHANNEL;
    int     SENSOR_TYPE;
    int     ID_RCV_DATA1;
    int     ID_RCV_DATA2;
    int     ID_RCV_ACC;
    int     ID_RCV_STAT;
    int     ID_RCV_INFO;
    int     ID_RCV_PARA;
}DB_FT;

// Database data type for IMU Sensor
typedef struct _DB_IMU_{
    int     BOARD_ID;
    int     SENSOR_ID;
    int     CAN_CHANNEL;
    int     SENSOR_TYPE;
    int     ID_RCV_DATA1;
    int     ID_RCV_DATA2;
    int     ID_RCV_STAT;
    int     ID_RCV_INFO;
    int     ID_RCV_PARA;
}DB_IMU;

// Database data type for Smart Power Controller
typedef struct _DB_SP_{
    int     BOARD_ID;
    int     CAN_CHANNEL;
    int     ID_RCV_DATA;
    int     ID_RCV_INFO;
    int     ID_SEND_GENERAL;
}DB_SP;

// Database data type for Optic Flow Sensor
typedef struct _DB_OF_{
    int     BOARD_ID;
    int     SENSOR_ID;
    int     CAN_CHANNEL;
    int     ID_RCV_DATA;
    int     ID_RCV_INFO;
}DB_OF;

// Database data type for PODO AL
typedef struct _DB_AL_{
    QString ALName;
    QString FileName;
    QString PathName;
}DB_AL;

//======================================================




//======================================================
typedef struct _RBCORE_JOINT_
{
    // Joint information from a board
    int             GainKp;
    int             GainKi;
    int             GainKd;
    int             EncoderResolution;
    int             PositiveDirection;
    int				AutoScale;
    int             Deadzone;
    int             SearchDirection;
    int             HomeSearchMode;
    int             LimitRevolution;
    double          OffsetAngle;
    double          LowerPositionLimit;
    double          UpperPositionLimit;
    int             MaxAcceleration;
    int             MaxVelocity;
    int             MaxPWM;
    int             MaxAccelerationDuringHome;
    int             MaxVelocityDuringHome;
    int				MaxVelocityDuringGoOffset;
    int             JAMmsTime;
    int             PWMmsTime;
    int             PWMDuty;
    int             JAMDuty;
    int             CurrentErrorLimitValue;
    int             BigInputErrorLimitValue;
    int             EncoderErrorLimitValue;

    double          Harmonic;
    double          PullyDrive;
    double          PullyDriven;
    double          PPR;    // Encoder pulse per one rotation(axis)
    double          ReductionRatio;
    double          FrictionParam1;
    double          FrictionParam2;

    // Joint status
    mSTAT           CurrentStatus;
    int             ControlMode;

    // Control reference
    double          Reference;
    double          RefPos;
    double          RefVel;
    double          RefVelConversionError;
    double          RefCurrent;
    double          RefPWM;
    double          RefPosOld;
    double          RefVelOld;
    double          RefCurrentOld;
    double          RefPWMOld;
    double          RefPosPreDefined[3];
    double          RefDefaultMotion[5];

    // Sensor information
    int             EncoderValue;
    double           CurrentPosition;
    double           CurrentVelocity;
    double           CurrentCurrent;
} RB_JOINT, *pRB_JOINT;
//======================================================
#endif // RBDATATYPE_H
