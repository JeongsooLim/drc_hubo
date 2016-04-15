// This BaseJoint.h is for DRC-HUBO

#ifndef BASEJOINT_H
#define BASEJOINT_H

#include <QVector>
#include <QString>

#define	NO_OF_JOINTS	36

class JointBaseClass;
typedef QVector<JointBaseClass*>	BaseJoints;



enum JointSequentialNumber
{
    RHY = 0, RHR, RHP, RKN, RAP, RAR,
    LHY, LHR, LHP, LKN, LAP, LAR,
    RSP, RSR, RSY, REB, RWY, RWP,
    LSP, LSR, LSY, LEB, LWY, LWP,
    NKY, NK1, NK2, WST,
    RF1, RF2, RF3, LF1, LF2, LF3,
    RWH, LWH
};


const QString JointNameList[NO_OF_JOINTS] = {
    "RHY", "RHR", "RHP", "RKN", "RAP", "RAR",
    "LHY", "LHR", "LHP", "LKN", "LAP", "LAR",
    "RSP", "RSR", "RSY", "REB", "RWY", "RWP",
    "LSP", "LSR", "LSY", "LEB", "LWY", "LWP",
    "NKY", "NK1", "NK2", "WST",
    "RF1", "RF2", "RF3", "LF1", "LF2", "LF3",
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
    {23,0},{23,1},{23,2},{12,0},
    {21,0},{21,1},{21,2},{22,0},{22,1},{22,2},
    {4,1}, {10,1}
};



class JointBaseClass
{
protected:
    QString		JointName;
    int			JointNumber;
    int			MCId;
    int			MCCh;
public:
    JointBaseClass()										{}
    JointBaseClass(QString str)								{JointName = str;}
    JointBaseClass(QString str, int num)					{JointName = str; JointNumber = num;}
    JointBaseClass(QString str, int num, int id, int ch)	{JointName = str; JointNumber = num; MCId = id; MCCh = ch;}
};


#endif // BASEJOINT_H

