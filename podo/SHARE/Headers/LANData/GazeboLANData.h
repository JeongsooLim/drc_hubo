
#ifndef GAZEBO_LAN_DATA_H
#define GAZEBO_LAN_DATA_H

#include "JointInformation.h"


enum GAZEBO_LAN_DATA_TYPE{
    GAZEBO_TYPE_NO = 0,
    GAZEBO_TYPE_JOINT,
    GAZEBO_TYPE_GAINOVERRIDE,
    GAZEBO_TYPE_HOME
};

typedef struct _DRC_GAZEBO_GO_CMD_
{
    int     joint;
    int     gain;
    int     timeMs;
}DRC_GAZEBO_GO_CMD;

typedef struct _DRC_GAZEBO_JOINT_REF_
{
    float   JointReference[NO_OF_JOINTS];
}DRC_GAZEBO_JOINT;


typedef struct _DRC_GAZEBO_TIME_
{
    int     sec;
    int     nsec;
}DRC_GAZEBO_TIME;

typedef struct _DRC_GAZEBO_FT_
{
    float force[3];     // fx, fy, fz
    float torque[3];    // mx, my, mz
}DRC_GAZEBO_FT;

typedef struct _DRC_GAZEBO_SENSOR_DATA_
{
    DRC_GAZEBO_TIME Sim_Time;
    DRC_GAZEBO_TIME ROS_Time;
    DRC_GAZEBO_FT   FTSensor[4];    // RAFT, LAFT, RWFT, LWFT
    float           IMUSensor[9];   // roll, pitch, yaw, rvel, pvel, yvel, accx, accy, accz
    float           JointCurrentPosition[NO_OF_JOINTS];
}DRC_GAZEBO_SENSOR;


#endif // GAZEBO_LAN_DATA_H
