#ifndef DEBRISMOTION_H
#define DEBRISMOTION_H


#define PITCH_INIT  53.25f


#include "BasicTrajectory.h"
#include "taskmotion.h"

#pragma once

#define NR_END 1

extern int WB_FLAG;
extern TaskMotion *WBmotion;
extern JointControlClass *joint;
extern pRBCORE_SHM_COMMAND     sharedCMD;
extern pRBCORE_SHM_REFERENCE   sharedREF;
extern pRBCORE_SHM_SENSOR      sharedSEN;
extern int IK_status;



typedef struct _DEBRIS_INFO_
{
    double  Center[3];
    double  Norm[3];
    double  Xdir[3];
    double  Ydir[3];

    double  Width;
    double  Length;

} DEBRIS_INFO, *pDEBRIS_INFO;

typedef enum{
	MTYPE_TEST,
    MTYPE_PICKRIGHTSIDE,
	MTYPE_STACKLEFTSIDE,
	MTYPE_SW_GRASP,
	MTYPE_SW_HOLD,
	MTYPE_SW_BACK

}ENUM_MOTIONTYPE;








typedef enum{
	MPHASE_Test_READY=0,
	MPHASE_Test_1,
	MPHASE_Test_2,
	MPHASE_Test_3,
	MPHASE_Test_END

}ENUM_MPHASE_TEST;

typedef enum{
    MPHASE_PICKRIGHTSIDE_0 = 0,
    MPHASE_PICKRIGHTSIDE_READY,
    MPHASE_PICKRIGHTSIDE_WSTTURN90,
	MPHASE_PICKRIGHTSIDE_RH,
	MPHASE_PICKRIGHTSIDE_LH,
	MPHASE_PICKRIGHTSIDE_FINOPEN,
	MPHASE_PICKRIGHTSIDE_RHDOCK,
	MPHASE_PICKRIGHTSIDE_LHDOCK,
	MPHASE_PICKRIGHTSIDE_RHGRASP,
	MPHASE_PICKRIGHTSIDE_LHGRASP,
	MPHASE_PICKRIGHTSIDE_FINCLOSE,
	MPHASE_PICKRIGHTSIDE_RHLIFT,
	MPHASE_PICKRIGHTSIDE_RHHOLD,
	MPHASE_PICKRIGHTSIDE_LHLIFT,
	MPHASE_PICKRIGHTSIDE_LHHOLD,

    MPHASE_PICKRIGHTSIDE_WSTTURN_90,
	MPHASE_PICKRIGHTSIDE_RHPUT,
    MPHASE_PICKRIGHTSIDE_RHNULL,
    MPHASE_PICKRIGHTSIDE_RHDOWN,
    MPHASE_PICKRIGHTSIDE_RHOFF,
    MPHASE_PICKRIGHTSIDE_RHUP,
    MPHASE_PICKRIGHTSIDE_RHON,

    MPHASE_PICKRIGHTSIDE_LHPUT,
    MPHASE_PICKRIGHTSIDE_LHNULL,
    MPHASE_PICKRIGHTSIDE_LHDOWN,
    MPHASE_PICKRIGHTSIDE_LHOFF,
    MPHASE_PICKRIGHTSIDE_LHUP,
    MPHASE_PICKRIGHTSIDE_LHON,

    MPHASE_PICKRIGHTSIDE_HANDREADY,
    MPHASE_PICKRIGHTSIDE_WSTTURN0,
    MPHASE_PICKRIGHTSIDE_HOME,



    // for #3 #4
    MPHASE1_PICKRIGHTSIDE_0,
    MPHASE1_PICKRIGHTSIDE_READY,
    MPHASE1_PICKRIGHTSIDE_WSTTURN90,
    MPHASE1_PICKRIGHTSIDE_RH,
    MPHASE1_PICKRIGHTSIDE_LH,
    MPHASE1_PICKRIGHTSIDE_FINOPEN,
    MPHASE1_PICKRIGHTSIDE_RHDOCK,
    MPHASE1_PICKRIGHTSIDE_LHDOCK,
    MPHASE1_PICKRIGHTSIDE_RHGRASP,
    MPHASE1_PICKRIGHTSIDE_LHGRASP,
    MPHASE1_PICKRIGHTSIDE_FINCLOSE,
    MPHASE1_PICKRIGHTSIDE_RHLIFT,
    MPHASE1_PICKRIGHTSIDE_RHHOLD,
    MPHASE1_PICKRIGHTSIDE_LHLIFT,
    MPHASE1_PICKRIGHTSIDE_LHHOLD,

    MPHASE1_PICKRIGHTSIDE_WSTTURN_90,
    MPHASE1_PICKRIGHTSIDE_RHPUT,
    MPHASE1_PICKRIGHTSIDE_RHNULL,
    MPHASE1_PICKRIGHTSIDE_RHDOWN,
    MPHASE1_PICKRIGHTSIDE_RHOFF,
    MPHASE1_PICKRIGHTSIDE_RHUP,
    MPHASE1_PICKRIGHTSIDE_RHON,

    MPHASE1_PICKRIGHTSIDE_LHPUT,
    MPHASE1_PICKRIGHTSIDE_LHNULL,
    MPHASE1_PICKRIGHTSIDE_LHDOWN,
    MPHASE1_PICKRIGHTSIDE_LHOFF,
    MPHASE1_PICKRIGHTSIDE_LHUP,
    MPHASE1_PICKRIGHTSIDE_LHON,

    MPHASE1_PICKRIGHTSIDE_HANDREADY,
    MPHASE1_PICKRIGHTSIDE_WSTTURN0,
    MPHASE1_PICKRIGHTSIDE_HOME,




	MPHASE_PICKRIGHTSIDE_END

}ENUM_MPHASE_PICKRIGHTSIDE;


typedef enum{
    MPHASE_STACKLEFTSIDE_0 = 0,
    MPHASE_STACKLEFTSIDE_READY,
    MPHASE_STACKLEFTSIDE_WSTTURN90,
    MPHASE_STACKLEFTSIDE_RH,
    MPHASE_STACKLEFTSIDE_LH,
    MPHASE_STACKLEFTSIDE_FINOPEN,
    MPHASE_STACKLEFTSIDE_RHDOCK,
    MPHASE_STACKLEFTSIDE_LHDOCK,
    MPHASE_STACKLEFTSIDE_RHGRASP,
    MPHASE_STACKLEFTSIDE_LHGRASP,
    MPHASE_STACKLEFTSIDE_FINCLOSE,
    MPHASE_STACKLEFTSIDE_RHLIFT,
    MPHASE_STACKLEFTSIDE_RHHOLD,
    MPHASE_STACKLEFTSIDE_LHLIFT,
    MPHASE_STACKLEFTSIDE_LHHOLD,

    MPHASE_STACKLEFTSIDE_WSTTURN_90,
    MPHASE_STACKLEFTSIDE_RHPUT,
    MPHASE_STACKLEFTSIDE_RHNULL,
    MPHASE_STACKLEFTSIDE_RHDOWN,
    MPHASE_STACKLEFTSIDE_RHOFF,
    MPHASE_STACKLEFTSIDE_RHUP,
    MPHASE_STACKLEFTSIDE_RHON,

    MPHASE_STACKLEFTSIDE_LHPUT,
    MPHASE_STACKLEFTSIDE_LHNULL,
    MPHASE_STACKLEFTSIDE_LHDOWN,
    MPHASE_STACKLEFTSIDE_LHOFF,
    MPHASE_STACKLEFTSIDE_LHUP,
    MPHASE_STACKLEFTSIDE_LHON,

    MPHASE_STACKLEFTSIDE_HANDREADY,
    MPHASE_STACKLEFTSIDE_WSTTURN0,
    MPHASE_STACKLEFTSIDE_HOME,



    // for #3 #4
    MPHASE1_STACKLEFTSIDE_0,
    MPHASE1_STACKLEFTSIDE_READY,
    MPHASE1_STACKLEFTSIDE_WSTTURN90,
    MPHASE1_STACKLEFTSIDE_RH,
    MPHASE1_STACKLEFTSIDE_LH,
    MPHASE1_STACKLEFTSIDE_FINOPEN,
    MPHASE1_STACKLEFTSIDE_RHDOCK,
    MPHASE1_STACKLEFTSIDE_LHDOCK,
    MPHASE1_STACKLEFTSIDE_RHGRASP,
    MPHASE1_STACKLEFTSIDE_LHGRASP,
    MPHASE1_STACKLEFTSIDE_FINCLOSE,
    MPHASE1_STACKLEFTSIDE_RHLIFT,
    MPHASE1_STACKLEFTSIDE_RHHOLD,
    MPHASE1_STACKLEFTSIDE_LHLIFT,
    MPHASE1_STACKLEFTSIDE_LHHOLD,

    MPHASE1_STACKLEFTSIDE_WSTTURN_90,
    MPHASE1_STACKLEFTSIDE_RHPUT,
    MPHASE1_STACKLEFTSIDE_RHNULL,
    MPHASE1_STACKLEFTSIDE_RHDOWN,
    MPHASE1_STACKLEFTSIDE_RHOFF,
    MPHASE1_STACKLEFTSIDE_RHUP,
    MPHASE1_STACKLEFTSIDE_RHON,

    MPHASE1_STACKLEFTSIDE_LHPUT,
    MPHASE1_STACKLEFTSIDE_LHNULL,
    MPHASE1_STACKLEFTSIDE_LHDOWN,
    MPHASE1_STACKLEFTSIDE_LHOFF,
    MPHASE1_STACKLEFTSIDE_LHUP,
    MPHASE1_STACKLEFTSIDE_LHON,

    MPHASE1_STACKLEFTSIDE_HANDREADY,
    MPHASE1_STACKLEFTSIDE_WSTTURN0,
    MPHASE1_STACKLEFTSIDE_HOME,




    MPHASE_STACKLEFTSIDE_END

}ENUM_MPHASE_STACKLEFTSIDE;



typedef enum{
	MPHASE_SW_GRASP_READY=0,
	MPHASE_SW_GRASP_1,
	MPHASE_SW_GRASP_2,
	MPHASE_SW_GRASP_3,
	MPHASE_SW_GRASP_4,
	MPHASE_SW_GRASP_5,
	MPHASE_SW_GRASP_END,

}ENUM_MPHASE_SW_GRASP;

typedef enum{
	MPHASE_SW_HOLD_READY=0,
	MPHASE_SW_HOLD_1,
	MPHASE_SW_HOLD_2,
	MPHASE_SW_HOLD_3,
	MPHASE_SW_HOLD_4,
	MPHASE_SW_HOLD_5,
	MPHASE_SW_HOLD_END,

}ENUM_MPHASE_SW_HOLD;

typedef enum{
	MPHASE_SW_BACK_READY=0,
	MPHASE_SW_BACK_1,
	MPHASE_SW_BACK_2,
	MPHASE_SW_BACK_3,
	MPHASE_SW_BACK_4,
	MPHASE_SW_BACK_5,
	MPHASE_SW_BACK_END,

}ENUM_MPHASE_SW_BACK;



class CDebrisMotion
{
public:
    double testval;

    bool MotionAllStopFlag;


public:
    CDebrisMotion();
    ~CDebrisMotion();

    DEBRIS_INFO *debrisinfo;
    int num_debris;

    // temp checker....
    int     CheckDebrisMotion(int motionType, double pos[], double quat[]);

    void    RefreshToCurrent(int mode);
    bool    CheckDebrisCollision();
//    TaskMotion      *debMotion;
//    CCollision      debCollision;
    double  ref_collision[17];
    bool CheckTerminated(bool col, bool wberror);
    void ChangeENDPhaseforCkechDebrisMotion(int mType);





    void RotPos(double _x, double _y, double _z, double _m[4][4]);
    void RotRx(double _th, double _m[4][4]);
    void RotRy(double _th, double _m[4][4]);
    void RotRz(double _th, double _m[4][4]);

    void mm_4x4(double _m1[4][4], double _m2[4][4], double _m[4][4]);
    void mm_4x1(double _m1[4][4], double _m2[4], double _m[4]);
    void mm_3x3(double _m1[3][3], double _m2[3][3], double _m[3][3]);
    void mm_3x1(double _m1[3][3], double _m2[3], double _m[3]);
    void DC2QT(double DC_3x3[3][3], double qt_4x1[4]);
    void QTCross(double qt1_4x1[4], double qt2_4x1[4], double result_4x1[4]);
    void RV2QT(double rv[4], double qt_4x1[4]);
    void QT2DC(double qt_4x1[4], double DC_3x3[3][3]);
    void QT2RV(double qt_4x1[4], double rv[4]);
    void QTinv(double qt_4x1[4], double result_4x1[4]);




    void Printf_mm_4x4(double _rot[4][4]);

    // ================================================================
    // Motion Generator.....
    void StartMotion();
    void MotionGenerator();

	void WBIKframe(TaskMotion *_wbmotion, int _mode);


    void Checkcnt();
    bool CheckENDphase();
    void Addcnt();

    void CheckIKerror();

    void GotoNextMotionPhase();
    void GotoENDMotionPhase();
    // -------------------------------------------------
        int  MotionType;
        bool bMotionGenerator;

        int  EndMotionPhase;
        int  timecnt;
        int  Goalcnt;
        int  MotionPhase;
        bool bReadFlag;
        int  MotionResult;
    // -------------------------------------------------
		double Destime;
		double Despos[3];       // des global
		double Desth[4];
		double Desrv[4];

		double Temprv1[4];
		double Temprv2[4];

		double Tempth1[4];
		double Tempth2[4];

		double Deselb;

		double Deswst;

		void MotionGen_Test();
		void MotionGen_RightSidePickup();
        void MotionGen_LeftSideStack();

		void MotionGen_SW_Grasp();
		void MotionGen_SW_Hold();
		void MotionGen_SW_Back();


    // -------------------------------------------------
    // ================================================================

    //
    float **matrix(unsigned int nrl, unsigned int nrh, unsigned int ncl, unsigned int nch);
    int inv(float **Ai, int n);
    void nrerror(char error_text[]);
    int *ivector(unsigned int nl, unsigned int nh);

    int	PushCANMessage(MANUAL_CAN MCData);

    int RBJointGainOverride(unsigned int _canch, unsigned int _bno, unsigned int _override1, unsigned int _override2, unsigned int _duration);

    int RBBoardSetSwitchingMode(unsigned int _canch, unsigned int _bno, int _mode);

    // ft mode...
    int RBwFTsensorNull(unsigned int _canch, unsigned int _bno);

    void RHFTFeedBack();
    void LHFTFeedBack();



    // safety check
    void EncCheckRFin();
    void EncCheckLFin();

    bool bEncCheck;
    bool bEncOK;

    bool bEncCheckRH;
    bool bEncOKRH;
    bool bEncCheckLH;
    bool bEncOKLH;

    bool bRHFTon, bLHFTon;




	void WSTmoveset(TaskMotion *_wbmotion, double _deswst, double _stime, int _mode);
	void PELmoveset(TaskMotion *_wbmotion, double _despel, double _stime, int _mode);

	void PELmoveset_0(TaskMotion *_wbmotion, double _despel, double _stime, int _mode);

	void RHPosmoveset(TaskMotion *_wbmotion, double _despos[3], double _stime, int _mode);
	void RHOrimoveset(TaskMotion *_wbmotion, double _desori[4], double _stime, int _mode);
	void LHPosmoveset(TaskMotion *_wbmotion, double _despos[3], double _stime, int _mode);
	void LHOrimoveset(TaskMotion *_wbmotion, double _desori[4], double _stime, int _mode);
	void RElbmoveset(TaskMotion *_wbmotion, double _desrelb, double _stime, int _mode);
	void LElbmoveset(TaskMotion *_wbmotion, double _deslelb, double _stime, int _mode);

    void COMmoveset(TaskMotion *_wbmotion, double _descomx, double _descomy, double _stime, int _mode);
    void PELZmoveset(TaskMotion *_wbmotion, double _despelz, double _stime, int _mode);


	double SW_Tp[3];
	double SW_Tw[4];

	class DebGrasp
	{
	public:
		double Pos[3];	// m xyz	// ground
		double Ori[2];	// deg	z x

		double Pos_r[3];	// reference frame on robot
		double Quat_r[4];
	};


	class DebStack
	{
	public:
		double Pos[3];	// m xyz	// ground
		double Ori[2];	// deg	z x

		double Pos_r[3];	// reference frame on robot
		double Quat_r[4];
	};


	DebGrasp DebG[4];
	DebStack DebS[4];


	DebGrasp TransFormDebInfo2Global(DebGrasp _deb);
    DebStack TransFormDebInfo2Global(DebStack _deb);
	double RElbCal(double _pos[3], double _quat[4], double _deswst);
	double LElbCal(double _pos[3], double _quat[4], double _deswst);


	void DockPoscal(double _tpos[3], double _tqt[4], double _distance, double _dpos[3]);    // in Z dir

	double Dock_Distance;
	double Wrist_Distance;
	double Palm_Distance;
	double Lift_Distance;
	double FT_Distance;

	double Dockpos[3];


};







#endif // DEBRISMOTION_H
