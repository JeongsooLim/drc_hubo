#ifndef TASKMOTION_H
#define TASKMOTION_H

#include "taskGeneral.h"
#include "BasicTrajectory.h"
//don;t know
#include "joint.h"
#include "BasicMatrix.h"
#include "../../SHARE/Libs/kine_drc_hubo3.h"
#include <string.h>



#define wheel_pitch 53.0f

enum WB_mode
{
    WBmode_NONE=0,
    WBmode_RULU,
    WBmode_RPLP,
    WBmode_RULP,
    WBmode_RPLU,
    WBmode_RGLG,
    WBmode_RGLU,
    WBmode_RULG
};
//int myWBIKmode;

using namespace rainbow;

class TaskMotion
{
    enum WB_TYPE
    {
        WB_POINT_TO_POINT,
        WB_ROTATE_VALVE
    };

public:
    explicit TaskMotion(pRBCORE_SHM_REFERENCE _shm_ref, pRBCORE_SHM_SENSOR _shm_sen, pRBCORE_SHM_COMMAND _shm_com, JointControlClass *_joint);
    ~TaskMotion();

private:
    pRBCORE_SHM_REFERENCE   Shm_ref;
    pRBCORE_SHM_SENSOR      Shm_sen;
    pRBCORE_SHM_COMMAND     Shm_com;
    JointControlClass   *Joint;

    WB_TYPE WBTYPE;


public:
    int         IK_status;
    int         myWBIKmode;

    KINE_DRC_HUBO3 kine_drc_hubo;

    //--------------------------------------------------------------------------------//
    TRHandler   wbPosRF[3];
    TRHandler   wbPosLF[3];
    TRHandler   wbPosRH[3];
    TRHandler   wbPosLH[3];
    TRHandler   wbPosWST;
    TRHandler   wbPosCOM[2];
    TRHandler   wbPosPelZ;
    TRHandler   wbPosELB[2];
    //TRHandler   wbPosBPitch;
    //--------------------------------------------------------------------------------//
    TRHandler   wbOriRF;
    TRHandler   wbOriLF;
    TRHandler   wbOriRH;
    TRHandler   wbOriLH;
    TRHandler   wbOriPel;

//    TRHandler   temp
    //--------------------------------------------------------------------------------//

    double      Qinit_34x1[34];  // inhyeok, 20141223
    double      Qin_34x1[34];
    double      Qout_34x1[34];
    double      Qub_34x1[34];
    double      Q_filt_34x1[34], Qd_filt_34x1[34], Qdd_filt_34x1[34];
    double      Q_enc_34x1[34];
    double      pCOM_2x1[2];
    double      pPelZ;
    double      pRF_3x1[3];
    double      pLF_3x1[3];
    double      pRH_3x1[3];
    double      pLH_3x1[3];

    double      qRF_4x1[4];
    double      qLF_4x1[4];
    double      qRH_4x1[4];
    double      qLH_4x1[4];
    double      qPEL_4x1[4];
    double      pWST_1x1[1];

    double      RElb_ang;
    double      LElb_ang;

    double      enc_pRH_3x1[3];
    double      enc_qRH_4x1[4];
    double      enc_Relb_ang;
    double      enc_pLH_3x1[3];
    double      enc_qLH_4x1[4];
    double      enc_Lelb_ang;
    void    enc_FK(void);


    double      des_pCOM_2x1[2];
    double      des_pPELz;
    double      des_rWST;
    double      des_pRF_3x1[3];
    double      des_pLF_3x1[3];
    double      des_pRH_3x1[3];
    double      des_pLH_3x1[3];

    double      des_qRF_4x1[4];
    double      des_qLF_4x1[4];
    double      des_qRH_4x1[4];
    double      des_qLH_4x1[4];
    double      des_qPEL_4x1[4];

    double      des_RElb_ang;
    double      des_LElb_ang;

    double      des_pCOM_3x1[3];

    double      init_rWST;

    bool        testflag;





    bool   bRElbcontrol;
    bool   bLElbcontrol;
    double Relbconst;
    double Lelbconst;

    double Relbx;
    double Relbxold;
    double Relbxoldold;

    double Relby;
    double Relbyold;

    double Lelbx;
    double Lelbxold;
    double Lelbxoldold;

    double Lelby;
    double Lelbyold;

    double JointRef_2[NO_OF_JOINTS];



    void    StopAll(void);
    void    ResetGlobalCoord(int RF_OR_LF_OR_PC);
    void    RefreshToCurrentReference(void);
    void    RefreshToCurrentReferenceUB(void);

    void addCOMInfo(double _xCOM, double _yCOM, double _sTime);
    void addCOMInfo(double _sTime);

    void addRFPosInfo(double _xLeg, double _yLeg, double _zLeg, double _sTime);
    void addRFPosInfo(double _sTime);

    void addLFPosInfo(double _xLeg, double _yLeg, double _zLeg, double _sTime);
    void addLFPosInfo(double _sTime);

    void addRHPosInfo(double _xArm, double _yArm, double _zArm, double _sTime);
    void addRHPosInfo(double _sTime);

    void addLHPosInfo(double _xArm, double _yArm, double _zArm, double _sTime);
    void addLHPosInfo(double _sTime);

    void addWSTPosInfo(double _wst, double _sTime);
    void addWSTPosInfo(double _sTime);

    void addPELPosInfo(double _pelz, double _sTime);
    void addPELPosInfo(double _sTime);

    void addPELOriInfo(doubles _quat, double _sTime);
    void addPELOriInfo(double _sTime);
    void addRFOriInfo(doubles _quat, double _sTime);
    void addRFOriInfo(double _sTime);
    void addLFOriInfo(doubles _quat, double _sTime);
    void addLFOriInfo(double _sTime);
    void addRHOriInfo(doubles _quat, double _sTime);
    void addRHOriInfo(double _sTime);
    void addLHOriInfo(doubles _quat, double _sTime);
    void addLHOriInfo(double _sTime);

    void updateAll();

    void WBIK();
    void WBIK_UB();

    double limit_Qd(double Qd, double Qdd, double Qd_max, double dt);

    void addRElbPosInfo(double _angle, double _sTime);
    void addRElbPosInfo(double _sTime);
    void addLElbPosInfo(double _angle, double _sTime);
    void addLElbPosInfo(double _sTime);

    void ResetGlobalCoord_2(int RF_OR_LF_OR_PC);

};




#endif // TASKMOTION_H

