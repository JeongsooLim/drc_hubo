#ifndef COMMONHEADER_H
#define COMMONHEADER_H

#include "BasicFiles/LANDialog.h"
#include "BasicFiles/RBDataBase.h"
#include "../SHARE/Headers/JointInformation.h"

extern LAN_PODO2GUI     PODO_DATA;
extern LANDialog        *pLAN;

const double PI = 3.1415927f;
const double R2D = 57.2957802f;
const double D2R = 0.0174533f;

inline float JointReference(int jnum){
    return PODO_DATA.CoreSEN.ENCODER[MC_GetID(jnum)][MC_GetCH(jnum)].CurrentReference;
}
inline float JointEncoder(int jnum){
    return PODO_DATA.CoreSEN.ENCODER[MC_GetID(jnum)][MC_GetCH(jnum)].CurrentPosition;
}

#endif // COMMONHEADER_H
