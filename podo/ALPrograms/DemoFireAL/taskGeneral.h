#ifndef TASKGENERAL_H
#define TASKGENERAL_H


#include "../../SHARE/Headers/RBSharedMemory.h"
#include "../../SHARE/Headers/ik_math2.h"
#include "../../SHARE/Headers/kine_drc_hubo2.h"

#include <stdio.h>
#include <math.h>
#include <unistd.h>

#define MODE_RELATIVE    0x00
#define MODE_ABSOLUTE    0x01
#define MODE_RELGLOBAL   0x02

#define HUBO2_SUCCESS           0
#define HUBO2_FAIL              1

#define DISABLE                 0
#define ENABLE                  1

#define     HUBO_RIGHT               0
#define     HUBO_LEFT                1
#define     HUBO_BOTH                2

//// WBIK Task space ------------------------------------------------------------------- //
//#define     TASK_RF             0
//#define     TASK_LF             1
//#define     TASK_RH             2
//#define     TASK_LH             3
//#define     TASK_PEL            4
//#define     TASK_COM            5
//#define     TASK_WST            6
//#define     NO_OF_TASK_SPACE    7
//// ----------------------------------------------------------------------------------- //


const double    OFFSET_ELB = -20.0;
const double    OFFSET_RSR = -15.0;
const double    OFFSET_LSR = 15.0;


enum ErrCode
{
    ERR_OK = 0,
    ERR_GOAL_TIME,
    ERR_ALREADY_MOVING,
    ERR_WRONG_MODE,
    ERR_WRONG_SELECTION
};

enum MovingStatus
{
    MOVE_DONE = 0,
    STILL_MOVING
};

#endif // TASKGENERAL_H
