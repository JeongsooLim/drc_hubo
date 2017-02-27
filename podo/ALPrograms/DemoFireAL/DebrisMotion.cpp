#include "DebrisMotion.h"

CDebrisMotion::CDebrisMotion()
{
    //debMotion = new TaskMotion(sharedData, joint);

    bEncCheck = true;
}

CDebrisMotion::~CDebrisMotion()
{
}


CDebrisMotion::DebGrasp CDebrisMotion::TransFormDebInfo2Global(DebGrasp _deb)
{
	double pel2G[4][4];
	double g2deb[4][4];
	double pel2deb[4][4];

	double tempDC[3][3];
	double tempM1[4][4];
	double tempM2[4][4];

	DebGrasp debout;


	RotPos(WBmotion->pRF_3x1[0], (WBmotion->pRF_3x1[1]+WBmotion->pLF_3x1[1])/2, WBmotion->pRF_3x1[2], pel2G);

	QT2DC(WBmotion->qRF_4x1, tempDC);
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			pel2G[i][j] = tempDC[i][j];

	//Printf_mm_4x4(pel2G);

	RotPos(_deb.Pos[0], _deb.Pos[1], _deb.Pos[2], g2deb);
	RotRz(_deb.Ori[0]*D2R, tempM1);
	mm_4x4(g2deb, tempM1, tempM2);

	RotRx(_deb.Ori[1]*D2R, tempM1);
	mm_4x4(tempM2, tempM1, g2deb);



	mm_4x4(pel2G, g2deb, pel2deb);
	//Printf_mm_4x4(pel2deb);
	for(int i=0;i<3;i++)
		debout.Pos_r[i] = pel2deb[i][3];

	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			tempDC[i][j] = pel2deb[i][j];
	DC2QT(tempDC, debout.Quat_r);

	return debout;

	//printf("aaa %.3f %.3f %.3f \n", _deb.Pos_r[0], _deb.Pos_r[1], _deb.Pos_r[2]);
}
CDebrisMotion::DebStack CDebrisMotion::TransFormDebInfo2Global(DebStack _deb)
{
    double pel2G[4][4];
    double g2deb[4][4];
    double pel2deb[4][4];

    double tempDC[3][3];
    double tempM1[4][4];
    double tempM2[4][4];

    DebStack debout;


    RotPos(WBmotion->pRF_3x1[0], (WBmotion->pRF_3x1[1]+WBmotion->pLF_3x1[1])/2, WBmotion->pRF_3x1[2], pel2G);

    QT2DC(WBmotion->qRF_4x1, tempDC);
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            pel2G[i][j] = tempDC[i][j];

    //Printf_mm_4x4(pel2G);

    RotPos(_deb.Pos[0], _deb.Pos[1], _deb.Pos[2], g2deb);
    RotRz(_deb.Ori[0]*D2R, tempM1);
    mm_4x4(g2deb, tempM1, tempM2);

    RotRx(_deb.Ori[1]*D2R, tempM1);
    mm_4x4(tempM2, tempM1, g2deb);



    mm_4x4(pel2G, g2deb, pel2deb);
    //Printf_mm_4x4(pel2deb);
    for(int i=0;i<3;i++)
        debout.Pos_r[i] = pel2deb[i][3];

    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            tempDC[i][j] = pel2deb[i][j];
    DC2QT(tempDC, debout.Quat_r);

    return debout;

    //printf("aaa %.3f %.3f %.3f \n", _deb.Pos_r[0], _deb.Pos_r[1], _deb.Pos_r[2]);
}

double CDebrisMotion::RElbCal(double _pos[3], double _quat[4], double _deswst)
{
	double Relbout;

	double minRelb = -130;
	double maxRelb = -30;
	double QQout34x1[34];
	double QQout7x1[7];
	double outQ7x1[7];

	int cnt_samping = 10;

	double Relb[cnt_samping];

	double Limit_upper[7];
	double Limit_lower[7];
	double Limit_wf[7];

	double min_matric;
	double cur_matric;

	bool blimit=false;

	Limit_upper[0] = 150.0; // sp
	Limit_upper[1] = -10.0;  // sr
	Limit_upper[2] = 145.0; // sy
	Limit_upper[3] = 10.0;  // eb
	Limit_upper[4] = 140.0; // wy
	Limit_upper[5] = 85.0;  // wp
	Limit_upper[6] = 160.0; // wy2

	Limit_lower[0] = -150.0;
	Limit_lower[1] = -130.0;
	Limit_lower[2] = -100.0;
	Limit_lower[3] = -160.0;
	Limit_lower[4] = -140.0;
	Limit_lower[5] = 0.0;
	Limit_lower[6] = -160.0;

	Limit_wf[0] = 0.5;
	Limit_wf[1] = 1.0;
	Limit_wf[2] = 7.0;
	Limit_wf[3] = 1.0;
	Limit_wf[4] = 2.0;
	Limit_wf[5] = 15.0;
	Limit_wf[6] = 0.5;

	for(int i = 0; i< cnt_samping; i++){
		Relb[i] = minRelb + (maxRelb-minRelb)/(cnt_samping-1)*i;
	}



	min_matric = 1000000000;
	Relbout = WBmotion->des_RElb_ang*R2D;

	for (int i =0; i< cnt_samping; i++){
		//_wbmotion->kine_drc_hubo.IK_UpperBody_Global_Orientation(_wbmotion->Qinit_34x1, _wbmotion->des_qPEL_4x1, _despos, _desth, Relb[i]*D2R, GLOBAL, _wbmotion->des_pLH_3x1, _wbmotion->des_qLH_4x1, _wbmotion->des_LElb_ang, LOCAL_UB, _wbmotion->des_rWST, QQout34x1);
		WBmotion->kine_drc_hubo.IK_WholeBody_Global(WBmotion->Qinit_34x1,
													 WBmotion->des_pCOM_2x1, WBmotion->des_pPELz, WBmotion->des_qPEL_4x1,
													 WBmotion->des_pRF_3x1, WBmotion->des_qRF_4x1,
													 WBmotion->des_pLF_3x1, WBmotion->des_qLF_4x1,
													 _pos, _quat, Relb[i]*D2R, GLOBAL,
													 WBmotion->des_pLH_3x1, WBmotion->des_qLH_4x1, WBmotion->des_LElb_ang, GLOBAL,
													 _deswst*D2R,
													 QQout34x1);

		QQout7x1[0] = QQout34x1[idRSP]*R2D;
		QQout7x1[1] = QQout34x1[idRSR]*R2D;
		QQout7x1[2] = QQout34x1[idRSY]*R2D;
		QQout7x1[3] = QQout34x1[idREB]*R2D;
		QQout7x1[4] = QQout34x1[idRWY]*R2D;
		QQout7x1[5] = QQout34x1[idRWP]*R2D;
		QQout7x1[6] = QQout34x1[idRWY2]*R2D;

		cur_matric = 0;
		blimit = false;
		for (int j =0;j<7;j++){
			cur_matric = cur_matric + Limit_wf[j]*fabs(  (QQout7x1[j] - ((Limit_upper[j]+Limit_lower[j])/2)) / fabs(Limit_upper[j]-Limit_lower[j]));

			if(Limit_upper[j] < QQout7x1[j] || Limit_lower[j] > QQout7x1[j] ){
				blimit = true;
				//printf(".. limit...... %d %d %.3f \n", i,j, Relb[i]);
			}

		}

		if(cur_matric < min_matric && blimit == false){
			min_matric = cur_matric;
			Relbout = Relb[i];
			//printf("%.3f %.3f \n", min_matric, Relbout);
			for (int j =0;j<7;j++){
				outQ7x1[j] = QQout7x1[j];
			}
		}

	}

	//printf("Relb result = %.3f \n", Relbout);
	return Relbout;

}

double CDebrisMotion::LElbCal(double _pos[3], double _quat[4], double _deswst)
{
	double Lelbout;

	double minLelb = 30;
	double maxLelb = 130;
	double QQout34x1[34];
	double QQout7x1[7];
	double outQ7x1[7];

	int cnt_samping = 16;

	double Lelb[cnt_samping];

	double Limit_upper[7];
	double Limit_lower[7];
	double Limit_wf[7];

	double min_matric;
	double cur_matric;

	bool blimit=false;

	Limit_upper[0] = 150.0; // sp
	Limit_upper[1] = 130.0;  // sr
	Limit_upper[2] = 100.0; // sy
	Limit_upper[3] = 10.0;  // eb
	Limit_upper[4] = 140.0; // wy
	Limit_upper[5] = 85.0;  // wp
	Limit_upper[6] = 160.0; // wy2

	Limit_lower[0] = -150.0;
	Limit_lower[1] = 10.0;
	Limit_lower[2] = -145.0;
	Limit_lower[3] = -160.0;
	Limit_lower[4] = -140.0;
	Limit_lower[5] = 0.0;
	Limit_lower[6] = -160.0;

	Limit_wf[0] = 0.5;
	Limit_wf[1] = 1.0;
	Limit_wf[2] = 7.0;
	Limit_wf[3] = 1.0;
	Limit_wf[4] = 2.0;
	Limit_wf[5] = 15.0;
	Limit_wf[6] = 0.5;

	for(int i = 0; i< cnt_samping; i++){
		Lelb[i] = minLelb + (maxLelb-minLelb)/(cnt_samping-1)*i;
	}



	min_matric = 1000000000;
	Lelbout = WBmotion->des_LElb_ang*R2D;

	for (int i =0; i< cnt_samping; i++){
		//_wbmotion->kine_drc_hubo.IK_UpperBody_Global_Orientation(_wbmotion->Qinit_34x1, _wbmotion->des_qPEL_4x1, _wbmotion->des_pRH_3x1, _wbmotion->des_qRH_4x1, _wbmotion->des_RElb_ang, LOCAL_UB, _despos, _desth, Lelb[i]*D2R, GLOBAL, _wbmotion->des_rWST, QQout34x1);
		WBmotion->kine_drc_hubo.IK_WholeBody_Global(WBmotion->Qinit_34x1,
													 WBmotion->des_pCOM_2x1, WBmotion->des_pPELz, WBmotion->des_qPEL_4x1,
													 WBmotion->des_pRF_3x1, WBmotion->des_qRF_4x1,
													 WBmotion->des_pLF_3x1, WBmotion->des_qLF_4x1,
													 WBmotion->des_pRH_3x1, WBmotion->des_qRH_4x1, WBmotion->des_RElb_ang, GLOBAL,
													 _pos, _quat, Lelb[i]*D2R, GLOBAL,
													 _deswst*D2R,
													 QQout34x1);

		QQout7x1[0] = QQout34x1[idLSP]*R2D;
		QQout7x1[1] = QQout34x1[idLSR]*R2D;
		QQout7x1[2] = QQout34x1[idLSY]*R2D;
		QQout7x1[3] = QQout34x1[idLEB]*R2D;
		QQout7x1[4] = QQout34x1[idLWY]*R2D;
		QQout7x1[5] = QQout34x1[idLWP]*R2D;
		QQout7x1[6] = QQout34x1[idLWY2]*R2D;

		cur_matric = 0;
		blimit = false;
		for (int j =0;j<7;j++){
			cur_matric = cur_matric + Limit_wf[j]*fabs(  (QQout7x1[j] - ((Limit_upper[j]+Limit_lower[j])/2)) / fabs(Limit_upper[j]-Limit_lower[j]));
			if(Limit_upper[j] < QQout7x1[j] || Limit_lower[j] > QQout7x1[j] ){
				blimit = true;
				//printf(".. limit...... %d %d %.3f \n", i,j, Relb[i]);
			}

		}

		if(cur_matric < min_matric && blimit == false){
			min_matric = cur_matric;
			Lelbout = Lelb[i];
			//printf("%.3f %.3f \n", min_matric, Relbout);
			for (int j =0;j<7;j++){
				outQ7x1[j] = QQout7x1[j];
			}
		}

	}

	//printf("Lelb result = %.3f \n", Lelbout);
	return Lelbout;
}

void CDebrisMotion::DockPoscal(double _tpos[3], double _tqt[4], double _distance, double _dpos[3])
{
	// calculate doc pos
	double Base2Debris[4][4];
	double Debris2Doc[4][4];
	double Base2Doc[4][4];
	double DebrisDC[3][3];
	double DebrisQT[4];
	double Distance_Debris2Doc;

	Distance_Debris2Doc = _distance;

	DebrisQT[0] = _tqt[0];
	DebrisQT[1] = _tqt[1];
	DebrisQT[2] = _tqt[2];
	DebrisQT[3] = _tqt[3];

	QT2DC(DebrisQT, DebrisDC);

	// base 2 debris matrix
	for (int i=0; i<3; i++)
	{
		for (int j=0; j<3; j++)
			Base2Debris[i][j] = DebrisDC[i][j];
	}
	Base2Debris[3][0] = 0;  Base2Debris[3][1] = 0;  Base2Debris[3][2] = 0;  Base2Debris[3][3] = 1;

	Base2Debris[0][3] = _tpos[0];
	Base2Debris[1][3] = _tpos[1];
	Base2Debris[2][3] = _tpos[2];

	// debris 2 doc matrix
	this->RotPos(0,0,Distance_Debris2Doc, Debris2Doc);

	// base 2 doc = base2debris x debris2doc
	this->mm_4x4(Base2Debris, Debris2Doc, Base2Doc);

	// doc pos = ...
	_dpos[0] = Base2Doc[0][3];
	_dpos[1] = Base2Doc[1][3];
	_dpos[2] = Base2Doc[2][3];

	// printf("        Tar pos  = %.2f %.2f %.2f \n", _tpos[0], _tpos[1], _tpos[2]);
	// printf("        Doc pos  = %.2f %.2f %.2f \n", _dpos[0], _dpos[1], _dpos[2]);
}











void CDebrisMotion::WBIKframe(TaskMotion *_wbmotion, int _mode)
{
	WB_FLAG = false;

	joint->RefreshToCurrentReference();

	_wbmotion->myWBIKmode = _mode;

	_wbmotion->ResetGlobalCoord(_mode);//Coord

	_wbmotion->StopAll();// Trajectory making ...

	_wbmotion->RefreshToCurrentReference();


//	printf("RH pos : %.3f %.3f %.3f \n", _wbmotion->pRH_3x1[0], _wbmotion->pRH_3x1[1], _wbmotion->pRH_3x1[2]);
//	printf("LH pos : %.3f %.3f %.3f \n", _wbmotion->pLH_3x1[0], _wbmotion->pLH_3x1[1], _wbmotion->pLH_3x1[2]);

//	printf("RF pos : %.3f %.3f %.3f \n", _wbmotion->pRF_3x1[0], _wbmotion->pRF_3x1[1], _wbmotion->pRF_3x1[2]);
//	printf("LF pos : %.3f %.3f %.3f \n", _wbmotion->pLF_3x1[0], _wbmotion->pLF_3x1[1], _wbmotion->pLF_3x1[2]);



	WB_FLAG = true;
}

void CDebrisMotion::Checkcnt(){

    if(this->timecnt == 0){
        this->bReadFlag = false;
    }

    if(this->Goalcnt-1 == this->timecnt){
        this->timecnt = -1;
        this->GotoNextMotionPhase();
    }
}

void CDebrisMotion::Addcnt(){
    this->timecnt++;
}

bool CDebrisMotion::CheckENDphase(){
    if(this->MotionPhase == EndMotionPhase){
        this->bMotionGenerator = false;

        if(this->MotionResult == 0)
            printf("\033[0;32mMotion End ! \033[0m\n");
        else
            printf("\033[0;31mMotion Fail %d ! \033[0m\n", this->MotionResult);

        joint->RefreshToCurrentReference();
        joint->SetAllMotionOwner();
		//WB_FLAG = false;
        return true;
    }
    return false;
}

void CDebrisMotion::GotoNextMotionPhase(){
    this->bReadFlag = true;

//    if(bEncCheck == true){
//        if(bEncCheckRH == true){
//            if(bEncOKRH == true){
//                this->MotionPhase++;
//                bEncOKRH = false;
//                printf("    Goto Next Phase RH \n");
//            }
//            else{
//                printf("    Repeat Cur Phase RH \n");
//            }
//        }
//        else if(bEncCheckLH == true){
//            if(bEncOKLH == true){
//                this->MotionPhase++;
//                bEncOKLH = false;
//                printf("    Goto Next Phase LH \n");
//            }
//            else{
//                printf("    Repeat Cur Phase LH \n");
//            }
//        }
//        else{
//            this->MotionPhase++;
//            printf("    Goto Next Phase .. \n");
//        }
//    }

//	else{
//		this->MotionPhase++;
//		printf("    Goto Next Phase \n");
//	}


    if(bEncCheckRH == true){
        if(bEncOKRH == true){
            this->MotionPhase++;
            bEncOKRH = false;
            printf("    Goto Next Phase RH \n");
        }
        else{
            printf("    Repeat Cur Phase RH \n");
        }
    }
    else if(bEncCheckLH == true){
        if(bEncOKLH == true){
            this->MotionPhase++;
            bEncOKLH = false;
            printf("    Goto Next Phase LH \n");
        }
        else{
            printf("    Repeat Cur Phase LH \n");
        }
    }
    else{
        this->MotionPhase++;
        printf("    Goto Next Phase .. \n");
    }


}

void CDebrisMotion::GotoENDMotionPhase(){
    this->MotionPhase = this->EndMotionPhase;
}



void CDebrisMotion::CheckIKerror()
{
//    if(WBmotion->IK_status != 0){
//        this->GotoENDMotionPhase();
//        this->MotionResult = WBmotion->IK_status;
//    }
}

void CDebrisMotion::RotPos(double _x, double _y, double _z, double _m[4][4])
{
    _m[0][0] = 1;   _m[0][1] = 0;   _m[0][2] = 0;   _m[0][3] = _x;
    _m[1][0] = 0;   _m[1][1] = 1;   _m[1][2] = 0;   _m[1][3] = _y;
    _m[2][0] = 0;   _m[2][1] = 0;   _m[2][2] = 1;   _m[2][3] = _z;
    _m[3][0] = 0;   _m[3][1] = 0;   _m[3][2] = 0;   _m[3][3] = 1;
}

void CDebrisMotion::RotRx(double _th, double _m[4][4])
{
    _m[0][0] = 1;    _m[0][1] = 0;            _m[0][2] = 0;            _m[0][3] = 0;
    _m[1][0] = 0;    _m[1][1] = cos(_th);     _m[1][2] = -sin(_th);    _m[1][3] = 0;
    _m[2][0] = 0;    _m[2][1] = sin(_th);     _m[2][2] = cos(_th);     _m[2][3] = 0;
    _m[3][0] = 0;    _m[3][1] = 0;            _m[3][2] = 0;            _m[3][3] = 1;
}

void CDebrisMotion::RotRy(double _th, double _m[4][4])
{
    _m[0][0] = cos(_th);     _m[0][1] = 0;    _m[0][2] = sin(_th);    _m[0][3] = 0;
    _m[1][0] = 0;            _m[1][1] = 1;    _m[1][2] = 0;            _m[1][3] = 0;
    _m[2][0] = -sin(_th);    _m[2][1] = 0;    _m[2][2] = cos(_th);    _m[2][3] = 0;
    _m[3][0] = 0;            _m[3][1] = 0;    _m[3][2] = 0;            _m[3][3] = 1;
}

void CDebrisMotion::RotRz(double _th, double _m[4][4])
{
    _m[0][0] = cos(_th);    _m[0][1] = -sin(_th); _m[0][2] = 0;    _m[0][3] = 0;
    _m[1][0] = sin(_th);    _m[1][1] = cos(_th);  _m[1][2] = 0;    _m[1][3] = 0;
    _m[2][0] = 0;           _m[2][1] = 0;         _m[2][2] = 1;    _m[2][3] = 0;
    _m[3][0] = 0;           _m[3][1] = 0;         _m[3][2] = 0;    _m[3][3] = 1;
}

void CDebrisMotion::mm_4x4(double _m1[4][4], double _m2[4][4], double _m[4][4])
{
    double _sum = 0;

    for (int i=0; i<4;i++)
    {
        for (int j=0; j<4;j++)
        {
            for (int k=0; k<4;k++)
            {
                _sum = _sum + _m1[i][k]*_m2[k][j];
            }
            _m[i][j] = _sum;
            _sum = 0;
        }
    }
}

void CDebrisMotion::mm_4x1(double _m1[4][4], double _m2[4], double _m[4])
{
    double _sum = 0;

    for (int i=0; i<4;i++)
    {
        for (int k=0; k<4;k++)
        {
            _sum = _sum + _m1[i][k]*_m2[k];
        }
        _m[i] = _sum;
        _sum = 0;
    }
}

void CDebrisMotion::mm_3x3(double _m1[3][3], double _m2[3][3], double _m[3][3])
{
    double _sum = 0;

    for (int i=0; i<3;i++)
    {
        for (int j=0; j<3;j++)
        {
            for (int k=0; k<3;k++)
            {
                _sum = _sum + _m1[i][k]*_m2[k][j];
            }
            _m[i][j] = _sum;
            _sum = 0;
        }
    }
}

void CDebrisMotion::mm_3x1(double _m1[3][3], double _m2[3], double _m[3])
{
    double _sum = 0;

    for (int i=0; i<3;i++)
    {
        for (int k=0; k<3;k++)
        {
            _sum = _sum + _m1[i][k]*_m2[k];
        }
        _m[i] = _sum;
        _sum = 0;
    }
}

void CDebrisMotion::DC2QT(double DC_3x3[3][3], double qt_4x1[4])
{
    double t = DC_3x3[0][0] + DC_3x3[1][1] + DC_3x3[2][2];
    double s;
    if(t > 0.f)
    {
        s = sqrtf(t+1.f)*2.f;
        qt_4x1[0] = 0.25f*s;
        qt_4x1[1] = (DC_3x3[2][1]-DC_3x3[1][2])/s;
        qt_4x1[2] = (DC_3x3[0][2]-DC_3x3[2][0])/s;
        qt_4x1[3] = (DC_3x3[1][0]-DC_3x3[0][1])/s;
    }
    else if((DC_3x3[0][0]>DC_3x3[1][1]) && (DC_3x3[0][0]>DC_3x3[2][2]))
    {
        s = sqrtf(1.f+DC_3x3[0][0]-DC_3x3[1][1]-DC_3x3[2][2])*2.f;
        qt_4x1[0] = (DC_3x3[2][1] - DC_3x3[1][2])/s;
        qt_4x1[1] = 0.25f*s;
        qt_4x1[2] = (DC_3x3[0][1] + DC_3x3[1][0])/s;
        qt_4x1[3] = (DC_3x3[0][2] + DC_3x3[2][0])/s;
    }
    else if(DC_3x3[1][1]>DC_3x3[2][2])
    {
        s = sqrtf(1.f+DC_3x3[1][1]-DC_3x3[0][0]-DC_3x3[2][2])*2.f;
        qt_4x1[0] = (DC_3x3[0][2] - DC_3x3[2][0])/s;
        qt_4x1[1] = (DC_3x3[0][1] + DC_3x3[1][0])/s;
        qt_4x1[2] = 0.25f*s;
        qt_4x1[3] = (DC_3x3[1][2] + DC_3x3[2][1])/s;
    }
    else
    {
        s = sqrtf(1.f+DC_3x3[2][2]-DC_3x3[0][0]-DC_3x3[1][1])*2.f;
        qt_4x1[0] = (DC_3x3[1][0] - DC_3x3[0][1])/s;
        qt_4x1[1] = (DC_3x3[0][2] + DC_3x3[2][0])/s;
        qt_4x1[2] = (DC_3x3[1][2] + DC_3x3[2][1])/s;
        qt_4x1[3] = 0.25f*s;
    }

    if(acos(qt_4x1[0])*2.f > PI)
    {
        qt_4x1[0] *= -1.f;
        qt_4x1[1] *= -1.f;
        qt_4x1[2] *= -1.f;
        qt_4x1[3] *= -1.f;
    }

}

void CDebrisMotion::QTCross(double qt1_4x1[4], double qt2_4x1[4], double result_4x1[4])
{
    result_4x1[0] = qt1_4x1[0]*qt2_4x1[0] - qt1_4x1[1]*qt2_4x1[1] - qt1_4x1[2]*qt2_4x1[2] - qt1_4x1[3]*qt2_4x1[3];
    result_4x1[1] = qt1_4x1[0]*qt2_4x1[1] + qt1_4x1[1]*qt2_4x1[0] + qt1_4x1[2]*qt2_4x1[3] - qt1_4x1[3]*qt2_4x1[2];
    result_4x1[2] = qt1_4x1[0]*qt2_4x1[2] + qt1_4x1[2]*qt2_4x1[0] - qt1_4x1[1]*qt2_4x1[3] + qt1_4x1[3]*qt2_4x1[1];
    result_4x1[3] = qt1_4x1[0]*qt2_4x1[3] + qt1_4x1[1]*qt2_4x1[2] - qt1_4x1[2]*qt2_4x1[1] + qt1_4x1[3]*qt2_4x1[0];
}

void CDebrisMotion::RV2QT(double rv[4], double qt_4x1[4])
{
    double temp = sqrtf(rv[1]*rv[1]+rv[2]*rv[2]+rv[3]*rv[3]);

    if(temp > 0.5f)
    {
        qt_4x1[0] = cosf(rv[0]/2.f);
        qt_4x1[1] = rv[1]/temp*sinf(rv[0]/2.f);
        qt_4x1[2] = rv[2]/temp*sinf(rv[0]/2.f);
        qt_4x1[3] = rv[3]/temp*sinf(rv[0]/2.f);
    }
    else
    {
        qt_4x1[0] = 1.f;
        qt_4x1[1] = 0.f;
        qt_4x1[2] = 0.f;
        qt_4x1[3] = 0.f;
    }

}

void CDebrisMotion::QT2RV(double qt_4x1[4], double rv[4])
{
    double temp;
    rv[0] = acosf(qt_4x1[0])*2.f;
    double EPS = (1.e-6);

    if(fabs(sinf(rv[0]/2.f)) < EPS)
    {
        temp = sqrtf(qt_4x1[1]*qt_4x1[1]+qt_4x1[2]*qt_4x1[2]+qt_4x1[3]*qt_4x1[3]+EPS);
        rv[1] = qt_4x1[1]/temp;
        rv[2] = qt_4x1[2]/temp;
        rv[3] = qt_4x1[3]/temp;
    }
    else
    {
        rv[1] = qt_4x1[1]/sinf(rv[0]/2.f);
        rv[2] = qt_4x1[2]/sinf(rv[0]/2.f);
        rv[3] = qt_4x1[3]/sinf(rv[0]/2.f);

        temp = sqrt(rv[1]*rv[1]+ rv[2]*rv[2]+ rv[3]*rv[3]);
        rv[1] /= temp;
        rv[2] /= temp;
        rv[3] /= temp;
    }

}

void CDebrisMotion::QT2DC(double qt_4x1[4], double DC_3x3[3][3])
{
    double q0 = qt_4x1[0];
    double q1 = qt_4x1[1];
    double q2 = qt_4x1[2];
    double q3 = qt_4x1[3];

    DC_3x3[0][0] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
    DC_3x3[0][1] = 2*(q1*q2-q0*q3);
    DC_3x3[0][2] = 2*(q1*q3+q0*q2);
    DC_3x3[1][0] = 2*(q1*q2+q0*q3);
    DC_3x3[1][1] = q0*q0 - q1*q1 + q2*q2 - q3*q3;
    DC_3x3[1][2] = 2*(q2*q3-q0*q1);
    DC_3x3[2][0] = 2*(q1*q3-q0*q2);
    DC_3x3[2][1] = 2*(q2*q3+q0*q1);
    DC_3x3[2][2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;
}

void CDebrisMotion::QTinv(double qt_4x1[4], double result_4x1[4])
{
    result_4x1[0] = qt_4x1[0]/((qt_4x1[0])*(qt_4x1[0])+(qt_4x1[1])*(qt_4x1[1])+(qt_4x1[2])*(qt_4x1[2])+(qt_4x1[3])*(qt_4x1[3]));
    result_4x1[1] = -qt_4x1[1]/((qt_4x1[0])*(qt_4x1[0])+(qt_4x1[1])*(qt_4x1[1])+(qt_4x1[2])*(qt_4x1[2])+(qt_4x1[3])*(qt_4x1[3]));
    result_4x1[2] = -qt_4x1[2]/((qt_4x1[0])*(qt_4x1[0])+(qt_4x1[1])*(qt_4x1[1])+(qt_4x1[2])*(qt_4x1[2])+(qt_4x1[3])*(qt_4x1[3]));
    result_4x1[3] = -qt_4x1[3]/((qt_4x1[0])*(qt_4x1[0])+(qt_4x1[1])*(qt_4x1[1])+(qt_4x1[2])*(qt_4x1[2])+(qt_4x1[3])*(qt_4x1[3]));
}

void CDebrisMotion::nrerror(char error_text[])
{
    printf("\nNumerical Recipes run-time error...\n");
    printf("%s\n",error_text);
}

float** CDebrisMotion::matrix(unsigned int nrl, unsigned int nrh, unsigned int ncl, unsigned int nch)
/* allocate a float matrix with subscript range m[nrl..nrh][ncl..nch] */
{
    unsigned int i, nrow=nrh-nrl+1,ncol=nch-ncl+1;
    float **m;

    /* allocate pointers to rows */
    m=(float **) malloc((size_t)((nrow+NR_END)*sizeof(float*)));
    if (!m) nrerror("allocation failure 1 in matrix()");
    m += NR_END;
    m -= nrl;

    /* allocate rows and set pointers to them */
    m[nrl]=(float *) malloc((size_t)((nrow*ncol+NR_END)*sizeof(float)));
    if (!m[nrl]) nrerror("allocation failure 2 in matrix()");
    m[nrl] += NR_END;
    m[nrl] -= ncl;

    for(i=nrl+1;i<=nrh;i++) m[i]=m[i-1]+ncol;

    /* return pointer to array of pointers to rows */
    return m;
}

#define SWAP(a,b) {temp=(a);(a)=(b);(b)=temp;}

int CDebrisMotion::inv(float **Ai, int n)
{
    double EPS = (1.e-6);
    int *indxc,*indxr,*ipiv;
    int i,icol,irow,j,k,l,ll;
    float big,dum,pivinv, temp;

    indxc=ivector(1,n);
    indxr=ivector(1,n);
    ipiv=ivector(1,n);
    for (j=1;j<=n;j++)
        ipiv[j]=0;

    for (i=1;i<=n;i++)
    {
        big=0.;
        for (j=1;j<=n;j++)
        {
            if (ipiv[j] != 1)
            {
                for (k=1;k<=n;k++)
                {
                    if (ipiv[k] == 0)
                    {
                        if (fabs(Ai[j][k]) >= big)
                        {
                            big=(float)fabs(Ai[j][k]);
                            irow=j;
                            icol=k;
                        }
                    }
                }
            }
        }
        ++(ipiv[icol]);
        if (irow != icol) {
            for (l=1;l<=n;l++)
                SWAP(Ai[irow][l],Ai[icol][l])
        }
        indxr[i]=irow;
        indxc[i]=icol;
        if (fabs(Ai[icol][icol]) <= EPS)
            return -1;
        pivinv=1.f/Ai[icol][icol];
        Ai[icol][icol]=1.f;
        for (l=1;l<=n;l++)
            Ai[icol][l] *= pivinv;
        for (ll=1;ll<=n;ll++)
            if (ll != icol) {
                dum=Ai[ll][icol];
                Ai[ll][icol]=0.0;
                for (l=1;l<=n;l++)
                    Ai[ll][l] -= Ai[icol][l]*dum;
            }
    }

    for (l=n;l>=1;l--) {
        if (indxr[l] != indxc[l])
            for (k=1;k<=n;k++)
                SWAP(Ai[k][indxr[l]],Ai[k][indxc[l]]);
    }
    free(ipiv);
    free(indxr);
    free(indxc);

    return 0;
}
#undef SWAP

int* CDebrisMotion::ivector(unsigned int nl, unsigned int nh)
/* allocate an int vector with subscript range v[nl..nh] */
{
    int *v;

    v=(int *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(int)));
    if (!v) nrerror("allocation failure in ivector()");
    return v-nl+NR_END;
}

void CDebrisMotion::Printf_mm_4x4(double _rot[4][4])
{
    printf("\n");
    for (int i=0;i<4;i++){
        for (int j=0;j<4;j++){
            printf("%.3f ", _rot[i][j]);
        }
        printf("\n");
    }
    printf("\n");
}



int	CDebrisMotion::PushCANMessage(MANUAL_CAN MCData){
    for(int i=0; i<MAX_MANUAL_CAN; i++){
        if(sharedCMD->ManualCAN[i].status == MANUALCAN_EMPTY){
            sharedCMD->ManualCAN[i].status = MANUALCAN_WRITING;
            sharedCMD->ManualCAN[i].channel = MCData.channel;
            sharedCMD->ManualCAN[i].id = MCData.id;
            sharedCMD->ManualCAN[i].dlc = MCData.dlc;
            for(int j=0; j<MCData.dlc; j++){
                sharedCMD->ManualCAN[i].data[j] = MCData.data[j];
            }
            sharedCMD->ManualCAN[i].status = MANUALCAN_NEW;
            return 0;
        }
    }
    printf("Fail to send Manual CAN..!! \n");
    return RB_FAIL;
}
int CDebrisMotion::RBJointGainOverride(unsigned int _canch, unsigned int _bno, unsigned int _override1, unsigned int _override2, unsigned int _duration){
    // MsgID                Byte0	Byte1	Byte2	Byte3	Byte4		Byte5
    // CANID_SEND_CMD		BNO		0x6F	OVER1	OVER2	DURATION	DURATION
    MANUAL_CAN	MCData;

    MCData.channel = _canch;
    MCData.id = COMMAND_CANID;
    MCData.dlc = 6;
    MCData.data[0] = _bno;
    MCData.data[1] = 0x6F;
    MCData.data[2] = _override1;
    MCData.data[3] = _override2;
    MCData.data[4] = (_duration & 0xFF);
    MCData.data[5] = ((_duration>>8) & (0xFF));

    return PushCANMessage(MCData);
}
int CDebrisMotion::RBBoardSetSwitchingMode(unsigned int _canch, unsigned int _bno, int _mode){
    // MsgID		Byte0	Byte1	Byte2
    // CMD_TXDF		BNO		0x13	_mode
    MANUAL_CAN	MCData;

    MCData.channel = _canch;
    MCData.id = COMMAND_CANID;
    MCData.dlc = 3;
    MCData.data[0] = _bno;
    MCData.data[1] = 0x13;
    MCData.data[2] = _mode;

    return PushCANMessage(MCData);
}
int CDebrisMotion::RBwFTsensorNull(unsigned int _canch, unsigned int _bno){
    // hand -> 0= Right 1=left 2=both
    // MsgID		Byte0	Byte1	Byte2
    // CMD_TXDF		BNO		0x81	_mode
    MANUAL_CAN	MCData;

    MCData.channel = _canch;
    MCData.id = COMMAND_CANID;
    MCData.dlc = 3;
    MCData.data[0] = _bno;
    MCData.data[1] = 0x81;
    MCData.data[2] = 0x00;
    // _mode = 0x00 : FT sensor
    // _mode = 0x04 : Inclinometers in FT sensor

    return PushCANMessage(MCData);
}

void CDebrisMotion::RHFTFeedBack()
{

    if(sharedSEN->FT[2].Fz > 10){
        printf("RH Break !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! \n");
		WBmotion->wbPosRH[0]->StopAndEraseAll();
		WBmotion->wbPosRH[1]->StopAndEraseAll();
		WBmotion->wbPosRH[2]->StopAndEraseAll();
        bRHFTon = false;
    }
}

void CDebrisMotion::LHFTFeedBack()
{
    if(sharedSEN->FT[3].Fz > 10){
        printf("LH Break !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! \n");
		WBmotion->wbPosLH[0]->StopAndEraseAll();
		WBmotion->wbPosLH[1]->StopAndEraseAll();
		WBmotion->wbPosLH[2]->StopAndEraseAll();
        bLHFTon = false;
    }
}

void CDebrisMotion::EncCheckRFin()
{
    if(sharedSEN->ENCODER[MC_GetID(RHAND)][MC_GetCH(RHAND)].CurrentPosition >38){
        bEncOKRH = true;
        bEncCheckRH = false;
        printf("R fin open! %.3f \n",sharedSEN->ENCODER[MC_GetID(RHAND)][MC_GetCH(RHAND)].CurrentPosition);
    }

}

void CDebrisMotion::EncCheckLFin()
{
    if(sharedSEN->ENCODER[MC_GetID(LHAND)][MC_GetCH(LHAND)].CurrentPosition >38){
        bEncOKLH = true;
        bEncCheckLH = false;
    }
}

// --------------------------------------------------------------------------------------------------------------------------------- //
bool CDebrisMotion::CheckDebrisCollision(){
//    ref_collision[1] = debMotion->Q_filt_34x1[idRSP];
//    ref_collision[2] = debMotion->Q_filt_34x1[idRSR];
//    ref_collision[3] = debMotion->Q_filt_34x1[idRSY];
//    ref_collision[4] = debMotion->Q_filt_34x1[idREB];
//    ref_collision[5] = debMotion->Q_filt_34x1[idRWY];
//    ref_collision[6] = debMotion->Q_filt_34x1[idRWP];
//    ref_collision[7] = debMotion->Q_filt_34x1[idRWY2];

//    ref_collision[8] = debMotion->Q_filt_34x1[idLSP];
//    ref_collision[9] = debMotion->Q_filt_34x1[idLSR];
//    ref_collision[10] = debMotion->Q_filt_34x1[idLSY];
//    ref_collision[11] = debMotion->Q_filt_34x1[idLEB];
//    ref_collision[12] = debMotion->Q_filt_34x1[idLWY];
//    ref_collision[13] = debMotion->Q_filt_34x1[idLWP];
//    ref_collision[14] = debMotion->Q_filt_34x1[idLWY2];

//    ref_collision[15] = debMotion->Q_filt_34x1[idWST];
//    ref_collision[16] = debMotion->Q_filt_34x1[idRHP];

	//debCollision.UpdateBoxInfo(ref_collision);
//    if(debCollision.CheckCollision()){
//        MotionResult = 100;
//        return true;
//    }
    return false;
}


bool CDebrisMotion::CheckTerminated(bool col, bool wberror)
{
    if(col == true || wberror == true)
        return true;
    else
        return false;
}

void CDebrisMotion::RefreshToCurrent(int mode){
    //joint->RefreshToCurrentReference();
//    debMotion->myWBIKmode = mode;
//    debMotion->ResetGlobalCoord(mode);
//    debMotion->StopAll();
//    debMotion->RefreshToCurrentReferenceUB();
}




void CDebrisMotion::WSTmoveset(TaskMotion *_wbmotion, double _deswst, double _stime, int _mode)
{
	if(_mode == MODE_ABSOLUTE){
		_wbmotion->addWSTPosInfo(_deswst, _stime);
//		if(_wbmotion == WBmotion){
//			joint->SetMoveJoint(WST, _wbmotion->init_rWST*R2D + _deswst, _stime*1000, MOVE_ABSOLUTE);
//		}
	}
	else if(_mode == MODE_RELATIVE){
        _wbmotion->addWSTPosInfo(_wbmotion->pWST_1x1[0]*R2D + _deswst, _stime);
//		if(_wbmotion == WBmotion){
//			joint->SetMoveJoint(WST, _deswst, _stime*1000, MOVE_RELATIVE);
//		}
	}
	else{
		printf("## WSTmoveset has no mode... \n");
	}
}

void CDebrisMotion::PELmoveset(TaskMotion *_wbmotion, double _despel, double _stime, int _mode)
{
	double peldesrv[4];
	double peldesqt[4];
	doubles peldesqts(4);
	double val_pel = -_despel;

	double pelcurqt[4];
	double peldesqt2[4];

	if(_mode == MODE_ABSOLUTE){
		peldesrv[0] = -val_pel*D2R;
		peldesrv[1] = 0;
		peldesrv[2] = 1;
		peldesrv[3] = 0;

		this->RV2QT(peldesrv, peldesqt);

		for (int i=0;i<4;i++)
			peldesqts[i] = peldesqt[i];


		_wbmotion->addPELOriInfo(peldesqts, _stime);

		if(_wbmotion == WBmotion){
//			joint->SetMoveJoint(RHP, -PITCH_INIT - val_pel, _stime*1000, MOVE_ABSOLUTE);
//			joint->SetMoveJoint(LHP, -PITCH_INIT - val_pel, _stime*1000, MOVE_ABSOLUTE);
		}

	}
	else if(_mode == MODE_RELATIVE){
		pelcurqt[0] = _wbmotion->qPEL_4x1[0];
		pelcurqt[1] = _wbmotion->qPEL_4x1[1];
		pelcurqt[2] = _wbmotion->qPEL_4x1[2];
		pelcurqt[3] = _wbmotion->qPEL_4x1[3];

		peldesrv[0] = -val_pel*D2R;
		peldesrv[1] = 0;
		peldesrv[2] = 1;
		peldesrv[3] = 0;

		this->RV2QT(peldesrv, peldesqt);

		this->QTCross(pelcurqt, peldesqt, peldesqt2);

		for (int i=0;i<4;i++)
			peldesqts[i] = peldesqt2[i];


		_wbmotion->addPELOriInfo(peldesqts, _stime);

		if(_wbmotion == WBmotion){
//			joint->SetMoveJoint(RHP, - val_pel, _stime*1000, MOVE_RELATIVE);
//			joint->SetMoveJoint(LHP, - val_pel, _stime*1000, MOVE_RELATIVE);
		}

	}
	else{
		printf("## PELmoveset has no mode... \n");
	}

}

void CDebrisMotion::PELmoveset_0(TaskMotion *_wbmotion, double _despel, double _stime, int _mode)
{
	double peldesrv[4];
	double peldesqt[4];
	doubles peldesqts(4);
	double val_pel = _despel;

	double pelcurqt[4];
	double peldesqt2[4];

	if(_mode == MODE_ABSOLUTE){
		peldesrv[0] = val_pel*D2R;
		peldesrv[1] = 0;
		peldesrv[2] = 1;
		peldesrv[3] = 0;

		this->RV2QT(peldesrv, peldesqt);

		for (int i=0;i<4;i++)
			peldesqts[i] = peldesqt[i];


		_wbmotion->addPELOriInfo(peldesqts, _stime);

		if(_wbmotion == WBmotion){
			joint->SetMoveJoint(RHP, -PITCH_INIT - val_pel, _stime*1000, MOVE_ABSOLUTE);
			joint->SetMoveJoint(LHP, -PITCH_INIT - val_pel, _stime*1000, MOVE_ABSOLUTE);
		}

	}
	else if(_mode == MODE_RELATIVE){
		pelcurqt[0] = _wbmotion->qPEL_4x1[0];
		pelcurqt[1] = _wbmotion->qPEL_4x1[1];
		pelcurqt[2] = _wbmotion->qPEL_4x1[2];
		pelcurqt[3] = _wbmotion->qPEL_4x1[3];

		peldesrv[0] = -val_pel*D2R;
		peldesrv[1] = 0;
		peldesrv[2] = 1;
		peldesrv[3] = 0;

		this->RV2QT(peldesrv, peldesqt);

		this->QTCross(pelcurqt, peldesqt, peldesqt2);

		for (int i=0;i<4;i++)
			peldesqts[i] = peldesqt2[i];


		_wbmotion->addPELOriInfo(peldesqts, _stime);

		if(_wbmotion == WBmotion){
			joint->SetMoveJoint(RHP, - val_pel, _stime*1000, MOVE_RELATIVE);
			joint->SetMoveJoint(LHP, - val_pel, _stime*1000, MOVE_RELATIVE);
		}

	}
	else{
		printf("## PELmoveset has no mode... \n");
	}

}

void CDebrisMotion::RHPosmoveset(TaskMotion *_wbmotion, double _despos[3], double _stime, int _mode)
{
	double _togo[3];

	if(_mode == MODE_ABSOLUTE){
		_togo[0] = _despos[0];
		_togo[1] = _despos[1];
		_togo[2] = _despos[2];

		_wbmotion->addRHPosInfo(_togo[0], _togo[1], _togo[2], _stime);

	}
	else if(_mode == MODE_RELATIVE){
		_togo[0] = _wbmotion->pRH_3x1[0] + _despos[0];
		_togo[1] = _wbmotion->pRH_3x1[1] + _despos[1];
		_togo[2] = _wbmotion->pRH_3x1[2] + _despos[2];

		_wbmotion->addRHPosInfo(_togo[0], _togo[1], _togo[2], _stime);
	}
	else{
		printf("## RHPosmoveset has no mode... \n");
	}
}

void CDebrisMotion::LHPosmoveset(TaskMotion *_wbmotion, double _despos[3], double _stime, int _mode)
{
	double _togo[3];

	if(_mode == MODE_ABSOLUTE){
		_togo[0] = _despos[0];
		_togo[1] = _despos[1];
		_togo[2] = _despos[2];

		_wbmotion->addLHPosInfo(_togo[0], _togo[1], _togo[2], _stime);

	}
	else if(_mode == MODE_RELATIVE){
		_togo[0] = _wbmotion->pLH_3x1[0] + _despos[0];
		_togo[1] = _wbmotion->pLH_3x1[1] + _despos[1];
		_togo[2] = _wbmotion->pLH_3x1[2] + _despos[2];

		_wbmotion->addLHPosInfo(_togo[0], _togo[1], _togo[2], _stime);
	}
	else{
		printf("## LHPosmoveset has no mode... \n");
	}
}

void CDebrisMotion::RHOrimoveset(TaskMotion *_wbmotion, double _desori[4], double _stime, int _mode)
{
	doubles _togo(4);

	double _togoqt[4];
	double _curqt[4];

	if(_mode == MODE_ABSOLUTE){

		_togo[0] = _desori[0];
		_togo[1] = _desori[1];
		_togo[2] = _desori[2];
		_togo[3] = _desori[3];

		_wbmotion->addRHOriInfo(_togo, _stime);
	}
	else if(_mode == MODE_RELATIVE){

		_curqt[0] = _wbmotion->qRH_4x1[0];
		_curqt[1] = _wbmotion->qRH_4x1[1];
		_curqt[2] = _wbmotion->qRH_4x1[2];
		_curqt[3] = _wbmotion->qRH_4x1[3];

		this->QTCross(_curqt, _desori, _togoqt);

		_togo[0] = _togoqt[0];
		_togo[1] = _togoqt[1];
		_togo[2] = _togoqt[2];
		_togo[3] = _togoqt[3];

		_wbmotion->addRHOriInfo(_togo, _stime);
	}
	else if(_mode == MODE_RELGLOBAL){

		_curqt[0] = _wbmotion->qRH_4x1[0];
		_curqt[1] = _wbmotion->qRH_4x1[1];
		_curqt[2] = _wbmotion->qRH_4x1[2];
		_curqt[3] = _wbmotion->qRH_4x1[3];

		this->QTCross(_desori, _curqt, _togoqt);

		_togo[0] = _togoqt[0];
		_togo[1] = _togoqt[1];
		_togo[2] = _togoqt[2];
		_togo[3] = _togoqt[3];

		_wbmotion->addRHOriInfo(_togo, _stime);
	}
	else{
		printf("## RHOrimoveset has no mode... \n");
	}
}

void CDebrisMotion::LHOrimoveset(TaskMotion *_wbmotion, double _desori[4], double _stime, int _mode)
{
	doubles _togo(4);

	double _togoqt[4];
	double _curqt[4];

	if(_mode == MODE_ABSOLUTE){

		_togo[0] = _desori[0];
		_togo[1] = _desori[1];
		_togo[2] = _desori[2];
		_togo[3] = _desori[3];

		_wbmotion->addLHOriInfo(_togo, _stime);
	}
	else if(_mode == MODE_RELATIVE){

		_curqt[0] = _wbmotion->qLH_4x1[0];
		_curqt[1] = _wbmotion->qLH_4x1[1];
		_curqt[2] = _wbmotion->qLH_4x1[2];
		_curqt[3] = _wbmotion->qLH_4x1[3];

		this->QTCross(_curqt, _desori, _togoqt);

		_togo[0] = _togoqt[0];
		_togo[1] = _togoqt[1];
		_togo[2] = _togoqt[2];
		_togo[3] = _togoqt[3];

		_wbmotion->addLHOriInfo(_togo, _stime);
	}
	else if(_mode == MODE_RELGLOBAL){

		_curqt[0] = _wbmotion->qLH_4x1[0];
		_curqt[1] = _wbmotion->qLH_4x1[1];
		_curqt[2] = _wbmotion->qLH_4x1[2];
		_curqt[3] = _wbmotion->qLH_4x1[3];

		this->QTCross(_desori, _curqt, _togoqt);

		_togo[0] = _togoqt[0];
		_togo[1] = _togoqt[1];
		_togo[2] = _togoqt[2];
		_togo[3] = _togoqt[3];

		_wbmotion->addLHOriInfo(_togo, _stime);
	}
	else{
		printf("## LHOrimoveset has no mode... \n");
	}
}

void CDebrisMotion::RElbmoveset(TaskMotion *_wbmotion, double _desrelb, double _stime, int _mode)
{
	double _togo;

	if(_mode == MODE_ABSOLUTE){
		_togo = _desrelb;

		_wbmotion->addRElbPosInfo(_togo, _stime);
	}
	else if(_mode == MODE_RELATIVE){
		_togo = _wbmotion->RElb_ang*R2D + _desrelb;

		_wbmotion->addRElbPosInfo(_togo, _stime);
	}
	else{
		printf("## RElbmoveset has no mode... \n");
	}
}

void CDebrisMotion::LElbmoveset(TaskMotion *_wbmotion, double _deslelb, double _stime, int _mode)
{
	double _togo;

	if(_mode == MODE_ABSOLUTE){
		_togo = _deslelb;

		_wbmotion->addLElbPosInfo(_togo, _stime);
	}
	else if(_mode == MODE_RELATIVE){
		_togo = _wbmotion->LElb_ang*R2D + _deslelb;

		_wbmotion->addLElbPosInfo(_togo, _stime);
	}
	else{
		printf("## LElbmoveset has no mode... \n");
	}

}

void CDebrisMotion::COMmoveset(TaskMotion *_wbmotion, double _descomx, double _descomy, double _stime, int _mode)
{
    double _togo1, _togo2;

    if(_mode == MODE_ABSOLUTE){
        _togo1 = _descomx;
        _togo2 = _descomy;

        _wbmotion->addCOMInfo(_togo1, _togo2, _stime);
    }
    else if(_mode == MODE_RELATIVE){
        _togo1 = _wbmotion->pCOM_2x1[0] + _descomx;
        _togo2 = _wbmotion->pCOM_2x1[1] + _descomy;

        _wbmotion->addCOMInfo(_togo1, _togo2, _stime);
    }
    else{
        printf("## COMmoveset has no mode... \n");
    }
}

void CDebrisMotion::PELZmoveset(TaskMotion *_wbmotion, double _despelz, double _stime, int _mode)
{
    double _togo;

    if(_mode == MODE_ABSOLUTE){
        _togo = _despelz;

        _wbmotion->addPELPosInfo(_togo, _stime);
    }
    else if(_mode == MODE_RELATIVE){
        _togo = _wbmotion->pPelZ + _despelz;

        _wbmotion->addPELPosInfo(_togo, _stime);
    }
    else{
        printf("## PELZmoveset has no mode... \n");
    }
}
















