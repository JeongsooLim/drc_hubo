#include "taskmotion.h"
#include "joint.h"
#include "../../SHARE/Libs/kine_drc_hubo3.h"
#include "../../SHARE/Libs/ik_math3.h"


//#include "DebrisMotion.h"

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
    kine_drc_hubo.upper_limit[idRWY2] = 6000*PI;
    kine_drc_hubo.lower_limit[idRWY2] = -6000*PI;
    kine_drc_hubo.upper_limit[idLWY2] = 6000*PI;
    kine_drc_hubo.lower_limit[idLWY2] = -6000*PI;

    kine_drc_hubo.upper_limit[idRKN] = 147*D2R;
    kine_drc_hubo.upper_limit[idLKN] = 147*D2R;
    kine_drc_hubo.lower_limit[idRAP] = -100*D2R;
    kine_drc_hubo.lower_limit[idLAP] = -100*D2R;

    kine_drc_hubo.upper_limit[idRSY] = 175*D2R;
    kine_drc_hubo.lower_limit[idRSY] = -140*D2R;
    kine_drc_hubo.upper_limit[idLSY] = 140*D2R;
    kine_drc_hubo.lower_limit[idLSY] = -175*D2R;

    kine_drc_hubo.upper_limit[idRWY] = 150*D2R;
    kine_drc_hubo.lower_limit[idRWY] = -150*D2R;

    kine_drc_hubo.upper_limit[idLWY] = 150*D2R;
    kine_drc_hubo.lower_limit[idLWY] = -150*D2R;

    kine_drc_hubo.upper_limit[idRSP] = 170*D2R;
    kine_drc_hubo.lower_limit[idRSP] = -170*D2R;
    kine_drc_hubo.upper_limit[idLSP] = 170*D2R;
    kine_drc_hubo.lower_limit[idLSP] = -170*D2R;


    kine_drc_hubo.L_PEL2PEL =0.2;
    kine_drc_hubo.L_SHOULDER2SHOULDER = 0.484;
    kine_drc_hubo.C_Torso[1] =0.;

    kine_drc_hubo.L_HAND=0.16;
    kine_drc_hubo.m_LeftLowerArm = 0;
    kine_drc_hubo.m_LeftUpperArm = 0;
    kine_drc_hubo.m_RightUpperArm = 0;
    kine_drc_hubo.m_RightLowerArm = 0;
    kine_drc_hubo.m_RightHand = 0;
    kine_drc_hubo.m_LeftHand = 0;
    kine_drc_hubo.iter_limit = 100;
    kine_drc_hubo.converge_criterium = 1e-6;
    kine_drc_hubo.orientation_weight = 0.01;

    kine_drc_hubo.Reset_Qinit();
    memcpy(Qinit_34x1, kine_drc_hubo.Qinit_34x1, 34*sizeof(double));
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

    //printf("before reset WST %.3f \n", Qin_34x1[idWST]*R2D);

    kine_drc_hubo.ResetGlobal(Qin_34x1, RF_OR_LF_OR_PC, NULL, Qout_34x1);

    //printf("after reset WST %.3f \n", Qout_34x1[idWST]*R2D);

    Qout_34x1[0] = 0;
    Qout_34x1[1] = 0;
    Qout_34x1[2] = 0;
    Qout_34x1[3] = 1;
    Qout_34x1[4] = 0;
    Qout_34x1[5] = 0;
    Qout_34x1[6] = 0;

    for(int i=0;i<=idWST;i++)
    {
        Q_filt_34x1[i] = Qout_34x1[i];
        Qd_filt_34x1[i] = 0;
        Qdd_filt_34x1[i] = 0;
    }
    WBTYPE = WB_POINT_TO_POINT;
}



void TaskMotion::ResetGlobalCoord_2(int RF_OR_LF_OR_PC)
{
    for(int i=RHY; i<=LAR; i++){
        Qin_34x1[idRHY+i] = this->JointRef_2[i]*D2R;
    }
    Qin_34x1[idWST] = this->JointRef_2[WST]*D2R;

    Qin_34x1[idRSP] = this->JointRef_2[RSP]*D2R;
    Qin_34x1[idRSR] = (this->JointRef_2[RSR]+OFFSET_RSR)*D2R;
    Qin_34x1[idRSY] = this->JointRef_2[RSY]*D2R;
    Qin_34x1[idREB] = (this->JointRef_2[REB]+OFFSET_ELB)*D2R;
    Qin_34x1[idRWY] = this->JointRef_2[RWY]*D2R;
    Qin_34x1[idRWP] = this->JointRef_2[RWP]*D2R;
    Qin_34x1[idRWY2] = this->JointRef_2[RWY2]*D2R;

    Qin_34x1[idLSP] = this->JointRef_2[LSP]*D2R;
    Qin_34x1[idLSR] = (this->JointRef_2[LSR]+OFFSET_LSR)*D2R;
    Qin_34x1[idLSY] = this->JointRef_2[LSY]*D2R;
    Qin_34x1[idLEB] = (this->JointRef_2[LEB]+OFFSET_ELB)*D2R;
    Qin_34x1[idLWY] = this->JointRef_2[LWY]*D2R;
    Qin_34x1[idLWP] = this->JointRef_2[LWP]*D2R;
    Qin_34x1[idLWY2] = this->JointRef_2[LWY2]*D2R;

    init_rWST = this->JointRef_2[WST]*D2R;

    Qin_34x1[idWST] = 0;

    kine_drc_hubo.ResetGlobal(Qin_34x1, RF_OR_LF_OR_PC, NULL, Qout_34x1);

    Qout_34x1[0] = 0;
    Qout_34x1[1] = 0;
    Qout_34x1[2] = 0;
    Qout_34x1[3] = 1;
    Qout_34x1[4] = 0;
    Qout_34x1[5] = 0;
    Qout_34x1[6] = 0;

    for(int i=0;i<=idWST;i++)
    {
        Q_filt_34x1[i] = Qout_34x1[i];
        Qd_filt_34x1[i] = 0;
        Qdd_filt_34x1[i] = 0;
    }
    WBTYPE = WB_POINT_TO_POINT;
}

void TaskMotion::enc_FK(void)
{
    int RH_frame;
    int LH_frame;
    if(myWBIKmode == WBmode_RULU){
        RH_frame = LOCAL_UB;
        LH_frame = LOCAL_UB;
    }else if(myWBIKmode == WBmode_RULP){
        RH_frame = LOCAL_UB;
        LH_frame = LOCAL_PELV;
    }else if(myWBIKmode == WBmode_RPLU){
        RH_frame = LOCAL_PELV;
        LH_frame = LOCAL_UB;
    }else if(myWBIKmode == WBmode_RPLP){
        RH_frame = LOCAL_PELV;
        LH_frame = LOCAL_PELV;
    }else{
        RH_frame = LOCAL_UB;
        LH_frame = LOCAL_UB;
    }
    kine_drc_hubo.FK_RightHand_Local(Q_enc_34x1, RH_frame, enc_pRH_3x1, enc_qRH_4x1, enc_Relb_ang);
    kine_drc_hubo.FK_LeftHand_Local(Q_enc_34x1, LH_frame, enc_pLH_3x1, enc_qLH_4x1, enc_Lelb_ang);
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
    WBTYPE = WB_POINT_TO_POINT;
}

void TaskMotion::RefreshToCurrentReferenceUB()
{
    int RH_frame;
    int LH_frame;
    if(myWBIKmode == WBmode_RULU){
        RH_frame = LOCAL_UB;
        LH_frame = LOCAL_UB;
    }else if(myWBIKmode == WBmode_RULP){
        RH_frame = LOCAL_UB;
        LH_frame = LOCAL_PELV;
    }else if(myWBIKmode == WBmode_RPLU){
        RH_frame = LOCAL_PELV;
        LH_frame = LOCAL_UB;
    }else if(myWBIKmode == WBmode_RPLP){
        RH_frame = LOCAL_PELV;
        LH_frame = LOCAL_PELV;
    }else if(myWBIKmode == WBmode_RGLG){
        RH_frame = GLOBAL;
        LH_frame = GLOBAL;
    }else if(myWBIKmode == WBmode_RGLU){
        RH_frame = GLOBAL;
        LH_frame = LOCAL_UB;
    }else if(myWBIKmode == WBmode_RULG){
        RH_frame = LOCAL_UB;
        LH_frame = GLOBAL;
    }else{
        RH_frame = LOCAL_UB;
        LH_frame = LOCAL_UB;
    }

    if(RH_frame == GLOBAL){
        kine_drc_hubo.FK_RightHand_Global(Q_filt_34x1, pRH_3x1, qRH_4x1, RElb_ang);
    }
    else{
        kine_drc_hubo.FK_RightHand_Local(Q_filt_34x1, RH_frame, pRH_3x1, qRH_4x1, RElb_ang);
    }

    if(LH_frame == GLOBAL){
        kine_drc_hubo.FK_LeftHand_Global(Q_filt_34x1, pLH_3x1, qLH_4x1, LElb_ang);
    }
    else{
        kine_drc_hubo.FK_LeftHand_Local(Q_filt_34x1, LH_frame, pLH_3x1, qLH_4x1, LElb_ang);
    }

    kine_drc_hubo.FK_RightFoot_Local(Q_filt_34x1, pRF_3x1, qRF_4x1);
    kine_drc_hubo.FK_LeftFoot_Local(Q_filt_34x1,  pLF_3x1, qLF_4x1);

    kine_drc_hubo.FK_COM_Local(Q_filt_34x1, pCOM_2x1);


    pWST_1x1[0] = Q_filt_34x1[idWST];
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
    wbPosWST->SetRetValue(Q_filt_34x1[idWST]*R2D);
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



    //inhyeok, 20141223//kine_drc_hubo.set_Q0(Q_filt_34x1);
    memcpy(Qinit_34x1, Q_filt_34x1, 34*sizeof(double));

    WBTYPE = WB_POINT_TO_POINT;



    Relbx = RElb_ang*R2D;
    Relbxold = Relbx;
    Relbxoldold = Relbx;

    Relby = RElb_ang*R2D;
    Relbyold = Relbx;

    Lelbx = LElb_ang*R2D;
    Lelbxold = Lelbx;
    Lelbxoldold = Lelbx;

    Lelby = LElb_ang*R2D;
    Lelbyold = Lelbx;

}
void TaskMotion::RefreshToCurrentReference(void){

	int RH_frame;
	int LH_frame;
	if(myWBIKmode == WBmode_RULU){
		RH_frame = LOCAL_UB;
		LH_frame = LOCAL_UB;
	}else if(myWBIKmode == WBmode_RULP){
		RH_frame = LOCAL_UB;
		LH_frame = LOCAL_PELV;
	}else if(myWBIKmode == WBmode_RPLU){
		RH_frame = LOCAL_PELV;
		LH_frame = LOCAL_UB;
	}else if(myWBIKmode == WBmode_RPLP){
		RH_frame = LOCAL_PELV;
		LH_frame = LOCAL_PELV;
	}else if(myWBIKmode == WBmode_RGLG){
		RH_frame = GLOBAL;
		LH_frame = GLOBAL;
	}else if(myWBIKmode == WBmode_RGLU){
		RH_frame = GLOBAL;
		LH_frame = LOCAL_UB;
	}else if(myWBIKmode == WBmode_RULG){
		RH_frame = LOCAL_UB;
		LH_frame = GLOBAL;
	}else{
		RH_frame = LOCAL_UB;
		LH_frame = LOCAL_UB;
	}

	if(RH_frame == GLOBAL){
		kine_drc_hubo.FK_RightHand_Global(Q_filt_34x1, pRH_3x1, qRH_4x1, RElb_ang);
	}
	else{
		kine_drc_hubo.FK_RightHand_Local(Q_filt_34x1, RH_frame, pRH_3x1, qRH_4x1, RElb_ang);
	}

	if(LH_frame == GLOBAL){
		kine_drc_hubo.FK_LeftHand_Global(Q_filt_34x1, pLH_3x1, qLH_4x1, LElb_ang);
	}
	else{
		kine_drc_hubo.FK_LeftHand_Local(Q_filt_34x1, LH_frame, pLH_3x1, qLH_4x1, LElb_ang);
	}



	kine_drc_hubo.FK_RightFoot_Global(Q_filt_34x1, pRF_3x1, qRF_4x1);
    kine_drc_hubo.FK_LeftFoot_Global(Q_filt_34x1, pLF_3x1, qLF_4x1);
//    kine_drc_hubo.FK_RightHand_Global(Q_filt_34x1, pRH_3x1, qRH_4x1, RElb_ang);
//    kine_drc_hubo.FK_LeftHand_Global(Q_filt_34x1, pLH_3x1, qLH_4x1, LElb_ang);
    kine_drc_hubo.FK_COM_Global(Q_filt_34x1, pCOM_2x1);
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
  //  printf("WSTWST %.3f %.3f \n", pWST_1x1[0], Q_filt_34x1[WST]);
//    wbPosWST->SetRetValue(Q_filt_34x1[WST]*D2R);
  //  printf("WSTWST %.3f %.3f \n", pWST_1x1[0], Q_filt_34x1[WST]);
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
    WBTYPE = WB_POINT_TO_POINT;
	memcpy(Qinit_34x1, Q_filt_34x1, 34*sizeof(double));

	updateAll();
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
    kine_drc_hubo.IK_WholeBody_Global(des_pCOM_2x1, des_pPELz, des_qPEL_4x1, des_pRF_3x1, des_qRF_4x1, des_pLF_3x1, des_qLF_4x1, des_pRH_3x1, des_qRH_4x1, des_RElb_ang, RH_ref_frame,des_pLH_3x1, des_qLH_4x1, des_LElb_ang, LH_ref_frame, des_rWST, Qout_34x1);
    */
    //kine_drc_hubo.IK_LowerBody_Global(Q_filt_34x1, des_pCOM_2x1, des_pPELz, des_qPEL_4x1, des_pRF_3x1, des_qRF_4x1, des_pLF_3x1, des_qLF_4x1, Qout_34x1);
    //inhyeok, 20141223//kine_drc_hubo.IK_LowerBody_Global(Q_filt_34x1, Qub_34x1, des_pCOM_3x1, des_qPEL_4x1, des_pRF_3x1, des_qRF_4x1, des_pLF_3x1, des_qLF_4x1, Qout_34x1);

	int RH_frame;
	int LH_frame;
	if(myWBIKmode == WBmode_RULU){
		RH_frame = LOCAL_UB;
		LH_frame = LOCAL_UB;
	}else if(myWBIKmode == WBmode_RULP){
		RH_frame = LOCAL_UB;
		LH_frame = LOCAL_PELV;
	}else if(myWBIKmode == WBmode_RPLU){
		RH_frame = LOCAL_PELV;
		LH_frame = LOCAL_UB;
	}else if(myWBIKmode == WBmode_RPLP){
		RH_frame = LOCAL_PELV;
		LH_frame = LOCAL_PELV;
	}else if(myWBIKmode == WBmode_RGLG){
		RH_frame = GLOBAL;
		LH_frame = GLOBAL;
	}else if(myWBIKmode == WBmode_RGLU){
		RH_frame = GLOBAL;
		LH_frame = LOCAL_UB;
	}else if(myWBIKmode == WBmode_RULG){
		RH_frame = LOCAL_UB;
		LH_frame = GLOBAL;
	}else{
		RH_frame = LOCAL_UB;
		LH_frame = LOCAL_UB;
	}

	//kine_drc_hubo.IK_LowerBody_Global(Qinit_34x1, Qub_34x1, des_pCOM_3x1, des_qPEL_4x1, des_pRF_3x1, des_qRF_4x1, des_pLF_3x1, des_qLF_4x1, Qout_34x1);
    kine_drc_hubo.IK_WholeBody_Global(Qinit_34x1,
                                      des_pCOM_2x1, des_pPELz, des_qPEL_4x1,
                                      des_pRF_3x1, des_qRF_4x1,
                                      des_pLF_3x1, des_qLF_4x1,
                                      des_pRH_3x1, des_qRH_4x1, des_RElb_ang, RH_frame,
                                      des_pLH_3x1, des_qLH_4x1, des_LElb_ang, LH_frame,
                                      des_rWST,
                                      Qout_34x1);





    //inhyeok, 20141223 //kine_drc_hubo.set_Q0(Qout_34x1);
    memcpy(Qinit_34x1,Qout_34x1,34*sizeof(double));

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

	if(RH_frame == GLOBAL){
		kine_drc_hubo.FK_RightHand_Global(Q_filt_34x1, pRH_3x1, qRH_4x1, RElb_ang);
	}
	else{
		kine_drc_hubo.FK_RightHand_Local(Q_filt_34x1, RH_frame, pRH_3x1, qRH_4x1, RElb_ang);
	}

	if(LH_frame == GLOBAL){
		kine_drc_hubo.FK_LeftHand_Global(Q_filt_34x1, pLH_3x1, qLH_4x1, LElb_ang);
	}
	else{
		kine_drc_hubo.FK_LeftHand_Local(Q_filt_34x1, LH_frame, pLH_3x1, qLH_4x1, LElb_ang);
	}
	 //kine_drc_hubo.FK_RightHand_Global(Q_filt_34x1, pRH_3x1, qRH_4x1, RElb_ang);
	 //kine_drc_hubo.FK_LeftHand_Global(Q_filt_34x1, pLH_3x1, qLH_4x1, LElb_ang);
     kine_drc_hubo.FK_RightFoot_Global(Q_filt_34x1, pRF_3x1, qRF_4x1);
     kine_drc_hubo.FK_LeftFoot_Global(Q_filt_34x1, pLF_3x1, qLF_4x1);
     kine_drc_hubo.FK_COM_Global(Q_filt_34x1, pCOM_2x1);

     pPelZ = Q_filt_34x1[idZ];//Qout_34x1[idZ];
     for(int i=0; i<4; i++){
         qPEL_4x1[i] = Q_filt_34x1[idQ0+i];
     }

     //inhyeok, 20141223 //kine_drc_hubo.set_Q0(Q_filt_34x1);
     memcpy(Qinit_34x1,Q_filt_34x1,34*sizeof(double));
}
void TaskMotion::WBIK_UB(){
    IK_RETURN_CODE ik_rtn;
    int RH_frame;
    int LH_frame;  
    if(myWBIKmode == WBmode_RULU){
        RH_frame = LOCAL_UB;
        LH_frame = LOCAL_UB;
    }else if(myWBIKmode == WBmode_RULP){
        RH_frame = LOCAL_UB;
        LH_frame = LOCAL_PELV;
    }else if(myWBIKmode == WBmode_RPLU){
        RH_frame = LOCAL_PELV;
        LH_frame = LOCAL_UB;
    }else if(myWBIKmode == WBmode_RPLP){
        RH_frame = LOCAL_PELV;
        LH_frame = LOCAL_PELV;
    }else if(myWBIKmode == WBmode_RGLG){
        RH_frame = GLOBAL;
        LH_frame = GLOBAL;
    }else if(myWBIKmode == WBmode_RGLU){
        RH_frame = GLOBAL;
        LH_frame = LOCAL_UB;
    }else if(myWBIKmode == WBmode_RULG){
        RH_frame = LOCAL_UB;
        LH_frame = GLOBAL;
    }else{
        RH_frame = LOCAL_UB;
        LH_frame = LOCAL_UB;
    }
    //inhyeok, 20141223 //kine_drc_hubo.IK_UpperBody_Local(des_pRH_3x1, des_qRH_4x1, des_RElb_ang, RH_frame, des_pLH_3x1, des_qLH_4x1, des_LElb_ang, LH_frame, des_rWST, Qout_34x1);
    //ik_rtn = kine_drc_hubo.IK_UpperBody_Local(Qinit_34x1, des_pRH_3x1, des_qRH_4x1, des_RElb_ang, RH_frame, des_pLH_3x1, des_qLH_4x1, des_LElb_ang, LH_frame, des_rWST, Qout_34x1);











    ik_rtn = kine_drc_hubo.IK_UpperBody_Global_Orientation(Qinit_34x1, des_qPEL_4x1,
                                                           des_pRH_3x1, des_qRH_4x1, des_RElb_ang, RH_frame, des_pLH_3x1, des_qLH_4x1, des_LElb_ang, LH_frame, des_rWST, Qout_34x1);



    if(ik_rtn.err_code.err_byte != 0){
        // printf("IK_upperbody_local returned error! %d ::", ik_rtn.err_code.err_bit, ik_rtn.err_code.err_byte);
        IK_status = ik_rtn.err_code.err_byte;
        if(ik_rtn.err_code.err_byte == 1){
            //printf("\033[1;31m joint limit \033[0m \n");
        }
        else if(ik_rtn.err_code.err_byte == 2){
            //printf("\033[1;31m workspace limit \033[0m \n");

        }
        else if(ik_rtn.err_code.err_byte == 3){
            //printf("\033[1;31m iteration limit \033[0m \n");

        }
        else if(ik_rtn.err_code.err_byte == 4){
            //printf("\033[1;31m ill quaternion \033[0m \n");

        }
        else if(ik_rtn.err_code.err_byte == 5){
            //printf("\033[1;31m big transition \033[0m \n");

        }
        else{
            //printf("?? \n");
        }

    }

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

    if(RH_frame == GLOBAL){
        kine_drc_hubo.FK_RightHand_Global(Q_filt_34x1, pRH_3x1, qRH_4x1, RElb_ang);
    }
    else{
        kine_drc_hubo.FK_RightHand_Local(Q_filt_34x1, RH_frame, pRH_3x1, qRH_4x1, RElb_ang);
    }

    if(LH_frame == GLOBAL){
        kine_drc_hubo.FK_LeftHand_Global(Q_filt_34x1, pLH_3x1, qLH_4x1, LElb_ang);
    }
    else{
        kine_drc_hubo.FK_LeftHand_Local(Q_filt_34x1, LH_frame, pLH_3x1, qLH_4x1, LElb_ang);
    }

     kine_drc_hubo.FK_RightFoot_Local(Q_filt_34x1, pRF_3x1, qRF_4x1);
     kine_drc_hubo.FK_LeftFoot_Local(Q_filt_34x1, pLF_3x1, qLF_4x1);
     kine_drc_hubo.FK_COM_Local(Q_filt_34x1, pCOM_2x1);

     pPelZ = Q_filt_34x1[idZ];
     for(int i=0; i<4; i++){
         qPEL_4x1[i] = Q_filt_34x1[idQ0+i];
     }
     //inhyeok, 20141223 //kine_drc_hubo.set_Q0(Q_filt_34x1);
     memcpy(Qinit_34x1,Q_filt_34x1,34*sizeof(double));
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
