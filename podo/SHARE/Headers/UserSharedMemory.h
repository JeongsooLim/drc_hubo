#ifndef USERSHAREDMEMORY_H
#define USERSHAREDMEMORY_H

#define USER_SHM_NAME         "USER_SHARED_MEMORY"

#ifndef __LAN_STRUCT_GENERAL_COMMAND_DEF__
#define __LAN_STRUCT_GENERAL_COMMAND_DEF__

typedef struct __LAN_STRUCT_GENERAL_COMMAND_
{
    char    param_c[10];
    int     param_i[10];
    float   param_f[10];
    int     cmd;
} LAN_GENERAL_COMMAND, *pLAN_GENERAL_COMMAND;

#endif

typedef struct _MOTION2GUI_
{
    float           obj_pos[3];
} MOTION2GUI, *pMOTION2GUI;

typedef struct _GUI2MOTION_
{
    LAN_GENERAL_COMMAND ros_cmd;
} GUI2MOTION, *pGUI2MOTION;

typedef struct _USER_SHM_
{
    MOTION2GUI  M2G;
    GUI2MOTION  G2M;

    double          WalkReadyCOM[3];

    float           odom_data[6];
    float           vel_cmd[2];
} USER_SHM, *pUSER_SHM;




#endif // USERSHAREDMEMORY_H
