#include "taskmotion.h"
#include "joint.h"




TaskMotion::TaskMotion(pRBCORE_SHM_REFERENCE _shm_ref, pRBCORE_SHM_SENSOR _shm_sen, pRBCORE_SHM_COMMAND _shm_com, JointControlClass *_joint){
    Shm_ref = _shm_ref;
    Shm_sen = _shm_sen;
    Shm_com = _shm_com;
    Joint = _joint;

    for(int i=0; i<3; i++){
        wbPosRF[i] = new TrajectoryHandler(ORDER_1, 0.005);
        wbPosLF[i] = new TrajectoryHandler(ORDER_1, 0.005);
        wbPosRH[i] = new TrajectoryHandler(ORDER_1, 0.005);
        wbPosLH[i] = new TrajectoryHandler(ORDER_1, 0.005);
    }
    wbPosWST = new TrajectoryHandler(ORDER_1, 0.005);
    for(int i=0; i<2; i++){
        wbPosCOM[i] = new TrajectoryHandler(ORDER_1, 0.005);
        wbPosELB[i] = new TrajectoryHandler(ORDER_1, 0.005);
    }
    wbPosPelZ = new TrajectoryHandler(ORDER_1, 0.005);

    wbOriRF = new TrajectoryHandler(ORDER_QUAT, 0.005);
    wbOriLF = new TrajectoryHandler(ORDER_QUAT, 0.005);
    wbOriRH = new TrajectoryHandler(ORDER_QUAT, 0.005);
    wbOriLH = new TrajectoryHandler(ORDER_QUAT, 0.005);
    wbOriPel = new TrajectoryHandler(ORDER_QUAT, 0.005);

    WBTYPE = WB_POINT_TO_POINT;
    kine_drc.upper_limit[idRWY2] = 6000*PI;
    kine_drc.lower_limit[idRWY2] = -6000*PI;
    kine_drc.upper_limit[idLWY2] = 6000*PI;
    kine_drc.lower_limit[idLWY2] = -6000*PI;

    kine_drc.upper_limit[idRKN] = 147*D2R;
    kine_drc.upper_limit[idLKN] = 147*D2R;
    kine_drc.lower_limit[idRAP] = -100*D2R;
    kine_drc.lower_limit[idLAP] = -100*D2R;

    kine_drc.upper_limit[idRSY] = 180*D2R;
    kine_drc.lower_limit[idRSY] = -95*D2R;
    kine_drc.upper_limit[idLSY] = 95*D2R;
    kine_drc.lower_limit[idLSY] = -180*D2R;

    kine_drc.L_HAND=0.16;
    kine_drc.C_Torso[1] = 0.;
    kine_drc.m_LeftLowerArm = 0;
    kine_drc.m_LeftUpperArm = 0;
    kine_drc.m_RightUpperArm = 0;
    kine_drc.m_RightLowerArm = 0;
    kine_drc.m_RightHand = 0;
    kine_drc.m_LeftHand = 0;
    kine_drc.iter_limit = 100;
    kine_drc.converge_criterium = 1e-6;
    kine_drc.orientation_weight = 0.01;
}

TaskMotion::~TaskMotion()
{
}

void TaskMotion::ResetGlobalCoord(int RF_OR_LF_OR_PC){
    for(int i=RHY; i<=LAR; i++){
        Qin_34x1[idRHY+i] = Joint->GetJointRefAngle(i)*D2R;
    }
    Qin_34x1[idWST] = Joint->GetJointRefAngle(WST)*D2R;

    Qin_34x1[idRSP] = Joint->GetJointRefAngle(RSP)*D2R;
    Qin_34x1[idRSR] = (Joint->GetJointRefAngle(RSR)+OFFSET_RSR)*D2R;
    Qin_34x1[idRSY] = Joint->GetJointRefAngle(RSY)*D2R;
    Qin_34x1[idREB] = (Joint->GetJointRefAngle(REB)+OFFSET_ELB)*D2R;
    Qin_34x1[idRWY] = Joint->GetJointRefAngle(RWY)*D2R;
    Qin_34x1[idRWP] = Joint->GetJointRefAngle(RWP)*D2R;
    Qin_34x1[idRWY2] = Joint->GetJointRefAngle(RWY2)*D2R;

    Qin_34x1[idLSP] = Joint->GetJointRefAngle(LSP)*D2R;
    Qin_34x1[idLSR] = (Joint->GetJointRefAngle(LSR)+OFFSET_LSR)*D2R;
    Qin_34x1[idLSY] = Joint->GetJointRefAngle(LSY)*D2R;
    Qin_34x1[idLEB] = (Joint->GetJointRefAngle(LEB)+OFFSET_ELB)*D2R;
    Qin_34x1[idLWY] = Joint->GetJointRefAngle(LWY)*D2R;
    Qin_34x1[idLWP] = Joint->GetJointRefAngle(LWP)*D2R;
    Qin_34x1[idLWY2] = Joint->GetJointRefAngle(LWY2)*D2R;


    kine_drc.ResetGlobal(Qin_34x1, RF_OR_LF_OR_PC, Qout_34x1);
    for(int i=0;i<=idWST;i++)
    {
        Q_filt_34x1[i] = Qout_34x1[i];
        Qd_filt_34x1[i] = 0;
        Qdd_filt_34x1[i] = 0;
    }
    WBTYPE = WB_POINT_TO_POINT;
}

void TaskMotion::StopAll(void){
    for(int i=0; i<3; i++){
        wbPosLH[i]->StopAndEraseAll();
        wbPosRH[i]->StopAndEraseAll();
        wbPosRF[i]->StopAndEraseAll();
        wbPosLF[i]->StopAndEraseAll();
    }
    wbPosPelZ->StopAndEraseAll();
    wbPosWST->StopAndEraseAll();
    wbPosCOM[0]->StopAndEraseAll();
    wbPosCOM[1]->StopAndEraseAll();

    wbOriLH->StopAndEraseAll();
    wbOriRH->StopAndEraseAll();
    wbOriRF->StopAndEraseAll();
    wbOriLF->StopAndEraseAll();
    wbOriPel->StopAndEraseAll();

    wbPosELB[0]->StopAndEraseAll();
    wbPosELB[1]->StopAndEraseAll();
    WBTYPE = WB_POINT_TO_POINT;
}

void TaskMotion::RefreshToCurrentReference(void){
    kine_drc.FK_RightFoot_Global(Q_filt_34x1, pRF_3x1, qRF_4x1);
    kine_drc.FK_LeftFoot_Global(Q_filt_34x1, pLF_3x1, qLF_4x1);
    kine_drc.FK_RightHand_Global(Q_filt_34x1, pRH_3x1, qRH_4x1, RElb_ang);
    kine_drc.FK_LeftHand_Global(Q_filt_34x1, pLH_3x1, qLH_4x1, LElb_ang);
    kine_drc.FK_COM_Global(Q_filt_34x1, pCOM_2x1);
    pPelZ = Q_filt_34x1[idZ];
    for(int i=0; i<3; i++){
        wbPosRF[i]->SetRetValue(pRF_3x1[i]);
        wbPosLF[i]->SetRetValue(pLF_3x1[i]);
        wbPosRH[i]->SetRetValue(pRH_3x1[i]);
        wbPosLH[i]->SetRetValue(pLH_3x1[i]);
    }
    wbPosPelZ->SetRetValue(Q_filt_34x1[idZ]);
    wbPosCOM[0]->SetRetValue(pCOM_2x1[0]);
    wbPosCOM[1]->SetRetValue(pCOM_2x1[1]);
    wbPosWST->SetRetValue(Q_filt_34x1[WST]*D2R);
    wbPosELB[0]->SetRetValue(RElb_ang*R2D);
    wbPosELB[1]->SetRetValue(LElb_ang*R2D);


    doubles tempRF, tempLF, tempRH, tempLH, tempPEL;
    for(int i=0; i<4; i++){
        tempRF.push_back(qRF_4x1[i]);
        tempLF.push_back(qLF_4x1[i]);
        tempRH.push_back(qRH_4x1[i]);
        tempLH.push_back(qLH_4x1[i]);
        tempPEL.push_back(Q_filt_34x1[idQ0+i]);
        qPEL_4x1[i] = Q_filt_34x1[idQ0+i];
    }
    wbOriRF->SetRetValue(tempRF);
    wbOriLF->SetRetValue(tempLF);
    wbOriRH->SetRetValue(tempRH);
    wbOriLH->SetRetValue(tempLH);
    wbOriPel->SetRetValue(tempPEL);
    kine_drc.set_Q0(Q_filt_34x1);
    WBTYPE = WB_POINT_TO_POINT;
}


void TaskMotion::addCOMInfo(double _xCOM, double _yCOM, double _sTime)
{
    TRInfo tempInfo;
    tempInfo = new TrajectoryCosine(_sTime, _xCOM);
    wbPosCOM[0]->AddTrajInfo(tempInfo);
    tempInfo = new TrajectoryCosine(_sTime, _yCOM);
    wbPosCOM[1]->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}
void TaskMotion::addCOMInfo(double _sTime){
    TRInfo tempInfo;
    tempInfo = new TrajectoryConst(_sTime);
    wbPosCOM[0]->AddTrajInfo(tempInfo);
    tempInfo = new TrajectoryConst(_sTime);
    wbPosCOM[1]->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}

void TaskMotion::addRFPosInfo(double _xLeg, double _yLeg, double _zLeg, double _sTime){
    TRInfo tempInfo;
    tempInfo = new TrajectoryCosine(_sTime, _xLeg);
    wbPosRF[0]->AddTrajInfo(tempInfo);
    tempInfo = new TrajectoryCosine(_sTime, _yLeg);
    wbPosRF[1]->AddTrajInfo(tempInfo);
    tempInfo = new TrajectoryCosine(_sTime, _zLeg);
    wbPosRF[2]->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}
void TaskMotion::addRFPosInfo(double _sTime){
    TRInfo tempInfo;
    tempInfo = new TrajectoryConst(_sTime);
    wbPosRF[0]->AddTrajInfo(tempInfo);
    tempInfo = new TrajectoryConst(_sTime);
    wbPosRF[1]->AddTrajInfo(tempInfo);
    tempInfo = new TrajectoryConst(_sTime);
    wbPosRF[2]->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}

void TaskMotion::addLFPosInfo(double _xLeg, double _yLeg, double _zLeg, double _sTime){
    TRInfo tempInfo;
    tempInfo = new TrajectoryCosine(_sTime, _xLeg);
    wbPosLF[0]->AddTrajInfo(tempInfo);
    tempInfo = new TrajectoryCosine(_sTime, _yLeg);
    wbPosLF[1]->AddTrajInfo(tempInfo);
    tempInfo = new TrajectoryCosine(_sTime, _zLeg);
    wbPosLF[2]->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}
void TaskMotion::addLFPosInfo(double _sTime){
    TRInfo tempInfo;
    tempInfo = new TrajectoryConst(_sTime);
    wbPosLF[0]->AddTrajInfo(tempInfo);
    tempInfo = new TrajectoryConst(_sTime);
    wbPosLF[1]->AddTrajInfo(tempInfo);
    tempInfo = new TrajectoryConst(_sTime);
    wbPosLF[2]->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}

void TaskMotion::addRHPosInfo(double _xArm, double _yArm, double _zArm, double _sTime){
    TRInfo tempInfo;
    tempInfo = new TrajectoryCosine(_sTime, _xArm);
    wbPosRH[0]->AddTrajInfo(tempInfo);
    tempInfo = new TrajectoryCosine(_sTime, _yArm);
    wbPosRH[1]->AddTrajInfo(tempInfo);
    tempInfo = new TrajectoryCosine(_sTime, _zArm);
    wbPosRH[2]->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}
void TaskMotion::addRHPosInfo(double _sTime){
    TRInfo tempInfo;
    tempInfo = new TrajectoryConst(_sTime);
    wbPosRH[0]->AddTrajInfo(tempInfo);
    tempInfo = new TrajectoryConst(_sTime);
    wbPosRH[1]->AddTrajInfo(tempInfo);
    tempInfo = new TrajectoryConst(_sTime);
    wbPosRH[2]->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}

void TaskMotion::addLHPosInfo(double _xArm, double _yArm, double _zArm, double _sTime){
    TRInfo tempInfo;
    tempInfo = new TrajectoryCosine(_sTime, _xArm);
    wbPosLH[0]->AddTrajInfo(tempInfo);
    tempInfo = new TrajectoryCosine(_sTime, _yArm);
    wbPosLH[1]->AddTrajInfo(tempInfo);
    tempInfo = new TrajectoryCosine(_sTime, _zArm);
    wbPosLH[2]->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}
void TaskMotion::addLHPosInfo(double _sTime){
    TRInfo tempInfo;
    tempInfo = new TrajectoryConst(_sTime);
    wbPosLH[0]->AddTrajInfo(tempInfo);
    tempInfo = new TrajectoryConst(_sTime);
    wbPosLH[1]->AddTrajInfo(tempInfo);
    tempInfo = new TrajectoryConst(_sTime);
    wbPosLH[2]->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}

void TaskMotion::addWSTPosInfo(double _wst, double _sTime){
    TRInfo tempInfo;
    tempInfo = new TrajectoryCosine(_sTime, _wst);
    wbPosWST->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}
void TaskMotion::addWSTPosInfo(double _sTime){
    TRInfo tempInfo;
    tempInfo = new TrajectoryConst(_sTime);
    wbPosWST->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}

void TaskMotion::addPELPosInfo(double _pelz, double _sTime){
    TRInfo tempInfo;
    tempInfo = new TrajectoryCosine(_sTime, _pelz);
    wbPosPelZ->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}
void TaskMotion::addPELPosInfo(double _sTime){
    TRInfo tempInfo;
    tempInfo = new TrajectoryConst(_sTime);
    wbPosPelZ->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}


void TaskMotion::addPELOriInfo(doubles _quat, double _sTime){
    TRInfo tempInfo = new TrajectoryQuatEuler(_sTime, _quat);//TrajectorySlerpNoExp
    wbOriPel->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}
void TaskMotion::addPELOriInfo(double _sTime){
    TRInfo tempInfo = new TrajectoryQuatConst(_sTime);
    wbOriPel->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}

void TaskMotion::addRFOriInfo(doubles _quat, double _sTime){
    TRInfo tempInfo = new TrajectoryQuatEuler(_sTime, _quat);
    wbOriRF->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}
void TaskMotion::addRFOriInfo(double _sTime){
    TRInfo tempInfo = new TrajectoryQuatConst(_sTime);
    wbOriRF->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}

void TaskMotion::addLFOriInfo(doubles _quat, double _sTime){
    TRInfo tempInfo = new TrajectoryQuatEuler(_sTime, _quat);
    wbOriLF->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}
void TaskMotion::addLFOriInfo(double _sTime){
    TRInfo tempInfo = new TrajectoryQuatConst(_sTime);
    wbOriLF->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}

void TaskMotion::addRHOriInfo(doubles _quat, double _sTime){
    TRInfo tempInfo = new TrajectoryQuatEuler(_sTime, _quat);
    wbOriRH->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}
void TaskMotion::addRHOriInfo(double _sTime){
    TRInfo tempInfo = new TrajectoryQuatConst(_sTime);
    wbOriRH->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}

void TaskMotion::addLHOriInfo(doubles _quat, double _sTime){
    TRInfo tempInfo = new TrajectoryQuatEuler(_sTime, _quat);
    wbOriLH->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}
void TaskMotion::addLHOriInfo(double _sTime){
    TRInfo tempInfo = new TrajectoryQuatConst(_sTime);
    wbOriLH->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}


void TaskMotion::updateAll(){
    if(WBTYPE==WB_POINT_TO_POINT)
    {
        for(int i=0; i<3; i++){
            doubles _rf = wbPosRF[i]->UpdateTrajectory();
            doubles _lf = wbPosLF[i]->UpdateTrajectory();
            doubles _rh = wbPosRH[i]->UpdateTrajectory();
            doubles _lh = wbPosLH[i]->UpdateTrajectory();

            des_pRF_3x1[i] = _rf[0];
            des_pLF_3x1[i] = _lf[0];
            des_pRH_3x1[i] = _rh[0];
            des_pLH_3x1[i] = _lh[0];
        }

        doubles _com1 = wbPosCOM[0]->UpdateTrajectory();
        doubles _com2 = wbPosCOM[1]->UpdateTrajectory();
        doubles _pelz = wbPosPelZ->UpdateTrajectory();
        doubles _wst = wbPosWST->UpdateTrajectory();
        doubles _relb = wbPosELB[0]->UpdateTrajectory();
        doubles _lelb = wbPosELB[1]->UpdateTrajectory();

        des_pCOM_2x1[0] = _com1[0];
        des_pCOM_2x1[1] = _com2[0];
        des_pPELz = _pelz[0];
        des_rWST = _wst[0]*D2R;//*R2D;
        des_RElb_ang = _relb[0]*D2R;
        des_LElb_ang = _lelb[0]*D2R;

        doubles _orf = wbOriRF->UpdateTrajectory();
        doubles _olf = wbOriLF->UpdateTrajectory();
        doubles _orh = wbOriRH->UpdateTrajectory();
        doubles _olh = wbOriLH->UpdateTrajectory();
        doubles _opel = wbOriPel->UpdateTrajectory();

        for(int i=0; i<4; i++){
            des_qRF_4x1[i] = _orf[i];
            des_qLF_4x1[i] = _olf[i];
            des_qRH_4x1[i] = _orh[i];
            des_qLH_4x1[i] = _olh[i];
            des_qPEL_4x1[i] = _opel[i];
        }
    }
}
double TaskMotion::limit_Qd(double Qd, double Qdd, double Qd_max, double dt)   //joint velocity limit function
{
    double qd_temp = Qd + Qdd*dt;
    double qdd_temp = Qdd;

    if(qd_temp > Qd_max)
        qdd_temp = (Qd_max-Qd)/dt;
    else if(qd_temp < -Qd_max)
        qdd_temp = (-Qd_max-Qd)/dt;

    return qdd_temp;
}
void TaskMotion::WBIK(){
    /*
    int RH_ref_frame = GLOBAL;//GLOBAL,LOCAL_PELV, LOCAL_UB
    int LH_ref_frame = GLOBAL;
    kine_drc.IK_WholeBody_Global(des_pCOM_2x1, des_pPELz, des_qPEL_4x1, des_pRF_3x1, des_qRF_4x1, des_pLF_3x1, des_qLF_4x1, des_pRH_3x1, des_qRH_4x1, des_RElb_ang, RH_ref_frame,des_pLH_3x1, des_qLH_4x1, des_LElb_ang, LH_ref_frame, des_rWST, Qout_34x1);
    */
    kine_drc.IK_LowerBody_Global(Q_filt_34x1, des_pCOM_2x1, des_pPELz, des_qPEL_4x1, des_pRF_3x1, des_qRF_4x1, des_pLF_3x1, des_qLF_4x1, Qout_34x1);
    kine_drc.set_Q0(Qout_34x1);

    const double DT = 0.005;
    const double KP = 20000;
    const double KD = 200;
    for(int k=idX; k<=idLAR;k++)
       { Q_filt_34x1[k] = Qout_34x1[k];}
    Q_filt_34x1[idWST] = Qout_34x1[idWST];
    for(int k=idRSP;k<=idLWY2;k++)
    {
        Qdd_filt_34x1[k] = limit_Qd(Qd_filt_34x1[k], -KD*Qd_filt_34x1[k] + KP*(Qout_34x1[k]-Q_filt_34x1[k]),2*PI, DT);
        Q_filt_34x1[k] += DT*Qd_filt_34x1[k] + 0.5*DT*DT*Qdd_filt_34x1[k];
        Qd_filt_34x1[k] += DT*Qdd_filt_34x1[k];
    }
    //----------------------------------------

     kine_drc.FK_RightHand_Global(Q_filt_34x1, pRH_3x1, qRH_4x1, RElb_ang);
     kine_drc.FK_LeftHand_Global(Q_filt_34x1, pLH_3x1, qLH_4x1, LElb_ang);
     kine_drc.FK_RightFoot_Global(Q_filt_34x1, pRF_3x1, qRF_4x1);
     kine_drc.FK_LeftFoot_Global(Q_filt_34x1, pLF_3x1, qLF_4x1);
     kine_drc.FK_COM_Global(Q_filt_34x1, pCOM_2x1);

     pPelZ = Q_filt_34x1[idZ];//Qout_34x1[idZ];
     for(int i=0; i<4; i++){
         qPEL_4x1[i] = Q_filt_34x1[idQ0+i];
     }

     kine_drc.set_Q0(Q_filt_34x1);
}

void TaskMotion::addRElbPosInfo(double _angle, double _sTime){
    TRInfo tempInfo;
    tempInfo = new TrajectoryCosine(_sTime, _angle);
    wbPosELB[0]->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}
void TaskMotion::addRElbPosInfo(double _sTime){
    TRInfo tempInfo;
    tempInfo = new TrajectoryConst(_sTime);
    wbPosELB[0]->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}
void TaskMotion::addLElbPosInfo(double _angle, double _sTime){
    TRInfo tempInfo;
    tempInfo = new TrajectoryCosine(_sTime, _angle);
    wbPosELB[1]->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}
void TaskMotion::addLElbPosInfo(double _sTime){
    TRInfo tempInfo;
    tempInfo = new TrajectoryConst(_sTime);
    wbPosELB[1]->AddTrajInfo(tempInfo);
    WBTYPE = WB_POINT_TO_POINT;
}
