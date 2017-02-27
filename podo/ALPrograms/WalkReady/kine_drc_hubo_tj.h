#ifndef KINE_DRC_HUBO_TJ_H
#define KINE_DRC_HUBO_TJ_H

class CKINE_DRC_HUBO_TJ
{
public:
    CKINE_DRC_HUBO_TJ();
    ~CKINE_DRC_HUBO_TJ();

public:
    long get_version() {return version;}
    void set_Q0(double Qinit_34x1[]){
        for(int i=0; i<34; i++)
            Q0_34x1[i] = Qinit_34x1[i];
    }
    void get_Q0(double Qinit_34x1[]){
        for(int i=0; i<34; i++)
            Qinit_34x1[i] = Q0_34x1[i];
    }

    int FK_RightFoot(double pRFoot_3x1[], double qRFoot_4x1[]);
    int FK_LeftFoot(double pLFoot_3x1[], double qLFoot_4x1[]);
    int FK_COM(double pCOM_3x1[]);

    int IK_LowerBody(const double des_pCOM_2x1[], double des_pPCz, const double des_qPEL_4x1[], const double des_pRF_3x1[], const double des_qRF_4x1[], const double des_pLF_3x1[], const double des_qLF_4x1[]);
    int IK_LowerBody(const double des_pCOM_3x1[], const double des_qPEL_4x1[], const double des_pRF_3x1[], const double des_qRF_4x1[], const double des_pLF_3x1[], const double des_qLF_4x1[]);

public:    
    double Q_34x1[34];
    int iter_limit;

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
    double L_ANKLE_PIT2ROL;

    //----- link COM coordinates
    double C_PELVIS[3];
    double C_TORSO[3];
    double C_RIGHT_UPPER_LEG[3];
    double C_RIGHT_LOWER_LEG[3];
    double C_RIGHT_FOOT[3];
    double C_LEFT_UPPER_LEG[3];
    double C_LEFT_LOWER_LEG[3];
    double C_LEFT_FOOT[3];
    double C_RIGHT_UPPER_ARM[3];
    double C_RIGHT_LOWER_ARM[3];
    double C_RIGHT_HAND[3];
    double C_LEFT_UPPER_ARM[3];
    double C_LEFT_LOWER_ARM[3];
    double C_LEFT_HAND[3];
    double C_RIGHT_ANKLE[3];
    double C_LEFT_ANKLE[3];

    //----- link mass
    double m_PELVIS;
    double m_TORSO;
    double m_RIGHT_UPPER_LEG;
    double m_RIGHT_LOWER_LEG;
    double m_RIGHT_FOOT;
    double m_LEFT_UPPER_LEG;
    double m_LEFT_LOWER_LEG;
    double m_LEFT_FOOT;
    double m_RIGHT_UPPER_ARM;
    double m_RIGHT_LOWER_ARM;
    double m_RIGHT_HAND;
    double m_LEFT_UPPER_ARM;
    double m_LEFT_LOWER_ARM;
    double m_LEFT_HAND;
    double m_RIGHT_ANKLE;
    double m_LEFT_ANKLE;



private:
    long version;
    double Q0_34x1[34];

private:
    int Jacob_RightFoot(const double Q_34x1[], double **jvRF_3x34, double **jwRF_3x34);
    int Jacob_LeftFoot(const double Q_34x1[], double **jvLF_3x34, double **jwLF_3x34);
    int Jacob_COM(const double Q_34x1[], double **jCOM_3x34);
    int FK_RightFoot(const double Q_34x1[], double pRFoot_3x1[], double qRFoot_4x1[]);
    int FK_LeftFoot(const double Q_34x1[], double pLFoot_3x1[], double qLFoot_4x1[]);
    int FK_COM(const double Q_34x1[], double pCOM_3x1[]);
};


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
#define PI			3.141592653589793
#define D2R			1.745329251994330e-2
#define R2D			5.729577951308232e1


#endif // KINE_DRC_HUBO_TJ_H
