#ifndef KINE_DRC_HUBO4_H
#define KINE_DRC_HUBO4_H

//------------------------- return code
#define IK_ERR          -1


//------------------------- Q_34x1[34] elements' index
#define idX         0
#define idY         1
#define idZ         2
#define idQ0        3
#define idQ1        4
#define idQ2        5
#define idQ3        6

#define idRHY       7
#define idRHR       8
#define idRHP       9
#define idRKN       10
#define idRAP       11
#define idRAR       12
#define idLHY       13
#define idLHR       14
#define idLHP       15
#define idLKN       16
#define idLAP       17
#define idLAR       18

#define idRSP		19   // right shoulder pitch
#define idRSR		20   // right shoulder roll
#define idRSY		21   // right shoulder yaw
#define idREB     	22   // right elbow
#define idRWY     	23   // right wrist yaw
#define idRWP     	24   // right wrist pitch
#define idRWY2     	25   // right wrist yaw2
#define idLSP		26   // left shoulder pitch
#define idLSR		27   // left shoulder roll
#define idLSY		28   // left shoulder yaw
#define idLEB		29   // left elbow
#define idLWY		30   // left wrist yaw
#define idLWP		31   // left wrist pitch
#define idLWY2  	32   // left wrist yaw2
#define idWST		33   // waist yaw


//-------------------------- misc.
#ifndef PI
#define PI			3.141592653589793
#endif

#ifndef D2R
#define D2R			1.745329251994330e-2
#endif

#ifndef R2D
#define R2D			5.729577951308232e1
#endif

//-------------------------- Reference frame
#define     GLOBAL          0
#define     LOCAL_PELV      1
#define     LOCAL_UB        2

typedef struct{
   union{
        struct{
            unsigned joint_limit:1;
            unsigned workspace_limit:1;
            unsigned iteration_limit:1;
            unsigned ill_quaternion:1;
            unsigned big_transition:1;
            unsigned rsvd:27;
        } err_bit;
        unsigned int err_byte;
   } err_code;
   unsigned int big_transition_joint_id;
   unsigned int limited_joint_id;
   unsigned int n_itertion;
} IK_RETURN_CODE;

class KINE_DRC_HUBO4
{
public:
    KINE_DRC_HUBO4();
    ~KINE_DRC_HUBO4();

public:
    long get_version() {return version;}
    void Reset_Qinit(void);

    IK_RETURN_CODE ResetGlobal(const double Q_34x1[], int RF_or_LF, const double qKnown_4x1[], double Q_return_34x1[]); // RF_or_LF: 1=LF, -1=RF, 0=PC

    IK_RETURN_CODE FK_RightFoot_Global(const double Q_34x1[], double pRF_3x1[], double qRF_4x1[]);
    IK_RETURN_CODE FK_LeftFoot_Global(const double Q_34x1[], double pLF_3x1[], double qLF_4x1[]);
    IK_RETURN_CODE FK_RightHand_Global(const double Q_34x1[], double pRH_3x1[], double qRH_4x1[], double &RElb_ang);
    IK_RETURN_CODE FK_LeftHand_Global(const double Q_34x1[], double pLH_3x1[], double qLH_4x1[], double &LElb_ang);

    IK_RETURN_CODE FK_RightFoot_Local(const double Q_34x1[], double pRF_3x1[], double qRF_4x1[]);
    IK_RETURN_CODE FK_LeftFoot_Local(const double Q_34x1[], double pLF_3x1[], double qLF_4x1[]);
    IK_RETURN_CODE FK_RightHand_Local(const double Q_34x1[], int RH_ref_frame, double pRH_3x1[], double qRH_4x1[], double &RElb_ang);
    IK_RETURN_CODE FK_LeftHand_Local(const double Q_34x1[], int LH_ref_frame, double pLH_3x1[], double qLH_4x1[], double &LElb_ang);

    IK_RETURN_CODE FK_COM_Global(const double Q_34x1[], double pCOM_3x1[]);
    IK_RETURN_CODE FK_COM_Local(const double Q_34x1[], double pCOM_3x1[]);

    IK_RETURN_CODE IK_LowerBody_Global(const double Qinit_34x1[], const double Q_ub_34x1[],
                                       const double des_pCOM_2x1[], double des_pPCz, const double des_qPEL_4x1[],
                                       const double des_pRF_3x1[], const double des_qRF_4x1[],
                                       const double des_pLF_3x1[], const double des_qLF_4x1[],
                                       double Q_return_34x1[]);

    IK_RETURN_CODE IK_LowerBody_Global(const double Qinit_34x1[], const double Q_ub_34x1[],
                                       const double des_pCOM_3x1[], const double des_qPEL_4x1[],
                                       const double des_pRF_3x1[], const double des_qRF_4x1[],
                                       const double des_pLF_3x1[], const double des_qLF_4x1[],
                                       double Q_return_34x1[]);

    IK_RETURN_CODE IK_WholeBody_Global(const double Qinit_34x1[],
                                       const double des_pCOM_2x1[], double des_pPCz, const double des_qPEL_4x1[],
                                       const double des_pRF_3x1[], const double des_qRF_4x1[],
                                       const double des_pLF_3x1[], const double des_qLF_4x1[],
                                       const double des_pRH_3x1[], const double des_qRH_4x1[], double des_RElb_ang, int RH_ref_frame, //reference frame: 0=global, 1=local pelvis, 2=local upper body
                                       const double des_pLH_3x1[], const double des_qLH_4x1[], double des_LElb_ang, int LH_ref_frame,
                                       double des_wst_ang,
                                       double Q_return_34x1[]);

    IK_RETURN_CODE IK_WholeBody_Global(const double Qinit_34x1[],
                                       const double des_pCOM_3x1[], const double des_qPEL_4x1[],
                                       const double des_pRF_3x1[], const double des_qRF_4x1[],
                                       const double des_pLF_3x1[], const double des_qLF_4x1[],
                                       const double des_pRH_3x1[], const double des_qRH_4x1[], double des_RElb_ang, int RH_ref_frame,
                                       const double des_pLH_3x1[], const double des_qLH_4x1[], double des_LElb_ang, int LH_ref_frame,
                                       double des_wst_ang,
                                       double Q_return_34x1[]);

    IK_RETURN_CODE IK_WholeBody_PC_Global(const double Qinit_34x1[],
                                          const double des_pPC_3x1[], const double des_qPEL_4x1[],
                                          const double des_pRF_3x1[], const double des_qRF_4x1[],
                                          const double des_pLF_3x1[], const double des_qLF_4x1[],
                                          const double des_pRH_3x1[], const double des_qRH_4x1[], double des_RElb_ang, int RH_ref_frame,
                                          const double des_pLH_3x1[], const double des_qLH_4x1[], double des_LElb_ang, int LH_ref_frame,
                                          double des_wst_ang,
                                          double Q_return_34x1[]);

    IK_RETURN_CODE IK_UpperBody_Local(const double Qinit_34x1[],
                                      const double des_pRH_3x1[], const double des_qRH_4x1[], double des_RElb_ang, int RH_ref_frame,
                                      const double des_pLH_3x1[], const double des_qLH_4x1[], double des_LElb_ang, int LH_ref_frame,
                                      double des_wst_ang,
                                      double Q_return_34x1[]);

    IK_RETURN_CODE IK_UpperBody_Global_Orientation(const double Qinit_34x1[],
                                                   const double des_qPEL_4x1[],
                                                   const double des_pRH_3x1[], const double des_qRH_4x1[], double des_RElb_ang, int RH_ref_frame,
                                                   const double des_pLH_3x1[], const double des_qLH_4x1[], double des_LElb_ang, int LH_ref_frame,
                                                   double des_wst_ang,
                                                   double Q_return_34x1[]);

    IK_RETURN_CODE IK_LowerBody_Local(const double des_pRF_3x1[], const double des_qRF_4x1[],
                                      const double des_pLF_3x1[], const double des_qLF_4x1[],
                                      double Q_return_34x1[]);

    IK_RETURN_CODE IK_WholeBody_Local(const double Qinit_34x1[],
                                      const double des_pRF_3x1[], const double des_qRF_4x1[],
                                      const double des_pLF_3x1[], const double des_qLF_4x1[],
                                      const double des_pRH_3x1[], const double des_qRH_4x1[], double des_RElb_ang, int RH_ref_frame,
                                      const double des_pLH_3x1[], const double des_qLH_4x1[], double des_LElb_ang, int LH_ref_frame,
                                      double des_wst_ang,
                                      double Q_return_34x1[]);


public:
    double leg_inner_workspace_limit;
    unsigned int iter_limit;
    double big_transition_limit;
    double converge_criterium;
    double Qinit_34x1[34];

public:
    //----- link length
    double L_PC2WST;
    double L_PEL2PEL;
    double L_UPPER_LEG;
    double L_LOWER_LEG;
    double L_ANKLE;
    double L_FOOT;
    double L_WST2SHOULDER;
    double L_SHOULDER2SHOULDER;
    double L_UPPER_ARM;
    double L_LOWER_ARM;
    double L_HAND;
    double L_ELB_OFFSET;

    //----- link COM coordinates
    double C_Pelvis[3];
    double C_Torso[3];
    double C_RightUpperArm[3];
    double C_RightLowerArm[3];
    double C_RightHand[3];
    double C_LeftUpperArm[3];
    double C_LeftLowerArm[3];
    double C_LeftHand[3];
    double C_RightUpperLeg[3];
    double C_RightLowerLeg[3];
    double C_RightFoot[3];
    double C_LeftUpperLeg[3];
    double C_LeftLowerLeg[3];
    double C_LeftFoot[3];

    //----- link mass
    double m_Pelvis;
    double m_Torso;
    double m_RightUpperArm;
    double m_RightLowerArm;
    double m_RightHand;
    double m_LeftUpperArm;
    double m_LeftLowerArm;
    double m_LeftHand;
    double m_RightUpperLeg;
    double m_RightLowerLeg;
    double m_RightFoot;
    double m_LeftUpperLeg;
    double m_LeftLowerLeg;
    double m_LeftFoot;

    double m_RightWrist;
    double m_LeftWrist;

    //----- joint limit
    double upper_limit[34];
    double lower_limit[34];

    const double AXIS_X[3];
    const double AXIS_Y[3];
    const double AXIS_Z[3];

private:
    long version;

public:
    int Jacob_RightFoot(const double q_34x1[], double jvRF_3x34[], double jwRF_3x34[]);
    int Jacob_LeftFoot(const double q_34x1[], double jvLF_3x34[], double jwLF_3x34[]);
    int Jacob_RightFoot_Local(const double q_34x1[], double jvRF_3x34[], double jwRF_3x34[]);
    int Jacob_LeftFoot_Local(const double q_34x1[], double jvLF_3x34[], double jwLF_3x34[]);
    int Jacob_RightArm(const double q_34x1[], double jvRWR_3x34[], double jwRH_3x34[], double jvRElb_3x34[]);
    int Jacob_LeftArm(const double q_34x1[], double jvLWR_3x34[], double jwLH_3x34[], double jvLElb_3x34[]);
    int Jacob_RightHand(const double q_34x1[], double jvRH_3x34[], double jwRH_3x34[], double jvRElb_3x34[]);
    int Jacob_LeftHand(const double q_34x1[], double jvLH_3x34[], double jwLH_3x34[], double jvLElb_3x34[]);
    int Jacob_COM(const double q_34x1[], double jCOM_3x34[]);

    int get_pElb(const double q_34x1[],
                 const double des_pRH_3x1[], const double des_qRH_4x1[], double des_RElb_ang,
                 const double des_pLH_3x1[], const double des_qLH_4x1[], double des_LElb_ang,
                 double pRElb_3x1[], double pRWR_3x1[], double pRH_3x1[], double qRH_4x1[], double &q_reb,
                 double pLElb_3x1[], double pLWR_3x1[], double pLH_3x1[], double qLH_4x1[], double &q_leb);
    unsigned int clamp_limit(double Q_34x1[]);


    IK_RETURN_CODE IK_RH_Geo_Local(const double Qinit_34x1[],
                                   const double des_pRH_3x1[], const double des_qRH_4x1[], double des_RElb_ang,
                                   double des_wst_ang,
                                   double Q_return_34x1[]);

    IK_RETURN_CODE IK_LH_Geo_Local(const double Qinit_34x1[],
                                   const double des_pLH_3x1[], const double des_qLH_4x1[], double des_LElb_ang,
                                   double des_wst_ang,
                                   double Q_return_34x1[]);

    IK_RETURN_CODE FK_RightFoot_Local(const double Q_34x1[],
                                      double pRF_3x1[], double qRF_4x1[], double pRPEL_3x1[], double pRKN_3x1[], double pRAP_3x1[]);
    IK_RETURN_CODE FK_LeftFoot_Local(const double Q_34x1[],
                                     double pLF_3x1[], double qLF_4x1[], double pLPEL_3x1[], double pLKN_3x1[],  double pLAP_3x1[]);

//    IK_RETURN_CODE IK_RF_Local(const double des_pRF_3x1[], const double des_qRF_4x1[],
//                               double Q_return_34x1[]);
//    IK_RETURN_CODE IK_LF_Local(const double des_pLF_3x1[], const double des_qLF_4x1[],
//                               double Q_return_34x1[]);

    IK_RETURN_CODE IK_RF_Geo_Local(const double des_pRF_3x1[], const double des_qRF_4x1[],
                                   double Q_return_34x1[]);
    IK_RETURN_CODE IK_LF_Geo_Local(const double des_pLF_3x1[], const double des_qLF_4x1[],
                                   double Q_return_34x1[]);

    IK_RETURN_CODE IK_RF_Geo_Global(const double pPC_3x1[], const double qPEL_4x1[],
                                    const double des_pRF_3x1[], const double des_qRF_4x1[],
                                    double Q_return_34x1[]);
    IK_RETURN_CODE IK_LF_Geo_Global(const double pPC_3x1[], const double qPEL_4x1[],
                                    const double des_pLF_3x1[], const double des_qLF_4x1[],
                                    double Q_return_34x1[]);

};


#endif
