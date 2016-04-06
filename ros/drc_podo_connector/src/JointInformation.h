// This JointInformation.h is for DRC-HUBO+

#ifndef JOINT_INFORMATION_H
#define JOINT_INFORMATION_H

#include <string>

enum JointSequentialNumber
{
    RHY = 0, RHR, RHP, RKN, RAP, RAR,
    LHY, LHR, LHP, LKN, LAP, LAR,
    RSP, RSR, RSY, REB, RWY, RWP,
    LSP, LSR, LSY, LEB, LWY, LWP,
    WST,
    RWY2, RHAND, LWY2, LHAND,
    RWH, LWH, NO_OF_JOINTS
};


const std::string JointNameList[NO_OF_JOINTS] = {
    "RHY", "RHR", "RHP", "RKN", "RAP", "RAR",
    "LHY", "LHR", "LHP", "LKN", "LAP", "LAR",
    "RSP", "RSR", "RSY", "REB", "RWY", "RWP",
    "LSP", "LSR", "LSY", "LEB", "LWY", "LWP",
    "WST",
    "RWY2", "RHAND", "LWY2", "LHAND",
    "RWH", "LWH"
};

const struct {
    int id;
    int ch;
} MC_ID_CH_Pairs[NO_OF_JOINTS] = {
    {0,0}, {1,0}, {2,0}, {3,0}, {4,0}, {5,0},
    {6,0}, {7,0}, {8,0}, {9,0}, {10,0},{11,0},
    {13,0}, {14,0}, {15,0}, {15,1}, {16,0},{16,1},
    {17,0},{18,0},{19,0},{19,1},{20,0},{20,1},
    {12,0},
    {21,0},{21,1},{22,0},{22,1},
    {4,1}, {10,1}
};

inline int MC_GetID(int jnum){
    return MC_ID_CH_Pairs[jnum].id;
}
inline int MC_GetCH(int jnum){
    return MC_ID_CH_Pairs[jnum].ch;
}

#endif // JOINT_INFORMATION_H
