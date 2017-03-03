#include "DebrisMotion.h"

void CDebrisMotion::StartMotion(){


	joint->RefreshToCurrentReference();
	joint->SetAllMotionOwner();

	this->MotionResult = 0;
	this->timecnt = 0;

	bRHFTon = false;
	bLHFTon = false;

    bEncCheckRH = false;
    bEncCheckLH = false;

	switch(this->MotionType){

	case MTYPE_TEST:

		WBIKframe(WBmotion, WBmode_RGLG);

        WBmotion->IK_status = 0;

		break;

    case MTYPE_PICKRIGHTSIDE:

        WBIKframe(WBmotion, WBmode_RULU);

        WBmotion->IK_status = 0;

        break;

    case MTYPE_STACKLEFTSIDE:

        WBIKframe(WBmotion, WBmode_RULU);

        WBmotion->IK_status = 0;

        break;

	case MTYPE_SW_GRASP:

		WBIKframe(WBmotion, WBmode_RULU);

		WBmotion->IK_status = 0;

		break;

	case MTYPE_SW_HOLD:

		WBIKframe(WBmotion, WBmode_RULU);

		WBmotion->IK_status = 0;

		break;

	case MTYPE_SW_BACK:

		WBIKframe(WBmotion, WBmode_RULU);

		WBmotion->IK_status = 0;

		break;
	}

	this->bMotionGenerator = true;
	this->bReadFlag = true;
	this->MotionPhase = 0;
}

void CDebrisMotion::MotionGenerator(){
   if(this->bMotionGenerator){

		if(this->bReadFlag){
			switch(this->MotionType){

			case MTYPE_TEST:
				MotionGen_Test();

				break;

			case MTYPE_PICKRIGHTSIDE:
				MotionGen_RightSidePickup();

				break;

            case MTYPE_STACKLEFTSIDE:
                MotionGen_LeftSideStack();

                break;

			case MTYPE_SW_GRASP:
				MotionGen_SW_Grasp();

				break;

			case MTYPE_SW_HOLD:
				MotionGen_SW_Hold();

				break;

			case MTYPE_SW_BACK:
				MotionGen_SW_Back();

				break;


			}
		}

		this->CheckIKerror();

		this->Checkcnt();
		this->Addcnt();
		bool isWBerror = false;
		isWBerror = this->CheckENDphase();
	}
}







void CDebrisMotion::MotionGen_Test()
{
	EndMotionPhase = MPHASE_Test_END;

	switch(this->MotionPhase){
	case MPHASE_Test_READY:
		this->Goalcnt = 600;
		printf("\033[1;33m MPHASE_Test_READY \033[0m\n");
		this->Destime = this->Goalcnt / 200.0;

		// wst
		Deswst = 0.0;
		this->WSTmoveset(WBmotion, Deswst, this->Destime, MODE_ABSOLUTE);

		// rh
		this->Despos[0] = 0.55;
		this->Despos[1] = -0.3;
		this->Despos[2] = 0.3;

		this->RHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

		this->Desrv[0] = -45*D2R;
		this->Desrv[1] = 0;
		this->Desrv[2] = 1;
		this->Desrv[3] = 0;
		this->RV2QT(this->Desrv, this->Desth);

		this->RHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

		Deselb = RElbCal(Despos, Desth, Deswst);
		this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

		// lh
		this->Despos[0] = 0.55;
		this->Despos[1] = 0.3;
		this->Despos[2] = 0.3;

		this->LHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

		this->Desrv[0] = -45*D2R;
		this->Desrv[1] = 0;
		this->Desrv[2] = 1;
		this->Desrv[3] = 0;
		this->RV2QT(this->Desrv, this->Desth);

		this->LHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

		Deselb = LElbCal(Despos, Desth, Deswst);
		this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);





		break;

	case MPHASE_Test_1:
		this->Goalcnt = 600;
		printf("\033[1;33m MPHASE_Test_1 \033[0m\n");
		//bEncCheck = true;

		this->Destime = this->Goalcnt / 200.0;

		//this->PELmoveset(WBmotion, 20.0, this->Destime, MODE_ABSOLUTE);
		break;

	case MPHASE_Test_2:
		this->Goalcnt = 600;
		printf("\033[1;33m MPHASE_Test_2 \033[0m\n");
		//bEncCheck = false;
		this->Destime = this->Goalcnt / 200.0;

		//this->WSTmoveset(WBmotion, 0.0, this->Destime, MODE_ABSOLUTE);

		break;

	case MPHASE_Test_3:
		this->Goalcnt = 600;
		printf("\033[1;33m MPHASE_Test_3 \033[0m\n");
		this->Destime = this->Goalcnt / 200.0;

		break;
	}
}

void CDebrisMotion::MotionGen_SW_Grasp()
{
	EndMotionPhase = MPHASE_SW_GRASP_END;

	switch(this->MotionPhase){
	case MPHASE_SW_GRASP_READY:
		this->Goalcnt = 400;
		printf("\033[1;33m MPHASE_SW_GRASP_READY \033[0m\n");
		this->Destime = this->Goalcnt / 200.0;


		// wst
		Deswst = 0.0;
		this->WSTmoveset(WBmotion, Deswst, this->Destime, MODE_ABSOLUTE);

		printf("%.3f %.3f %.3f \n", WBmotion->pRH_3x1[0], WBmotion->pRH_3x1[1], WBmotion->pRH_3x1[2]);
		this->QT2RV(WBmotion->qRH_4x1, this->Temprv1);
		printf("%.3f %.3f %.3f %.3f\n", this->Temprv1[0], this->Temprv1[1], this->Temprv1[2], this->Temprv1[3]);
		printf("%.2f \n", WBmotion->RElb_ang*R2D);

		// rh
		this->Despos[0] = WBmotion->pRH_3x1[0];
		this->Despos[1] = WBmotion->pRH_3x1[1];
		this->Despos[2] = WBmotion->pRH_3x1[2];

		this->RHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

		this->Desth[0] = WBmotion->qRH_4x1[0];
		this->Desth[1] = WBmotion->qRH_4x1[1];
		this->Desth[2] = WBmotion->qRH_4x1[2];
		this->Desth[3] = WBmotion->qRH_4x1[3];

		this->RHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

		Deselb = RElbCal(Despos, Desth, Deswst);
//		printf("%.2f \n", Deselb*R2D);
		this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);



		break;


	case MPHASE_SW_GRASP_1:
		this->Goalcnt = 600;
		printf("\033[1;33m MPHASE_SW_GRASP_1 \033[0m\n");
		this->Destime = this->Goalcnt / 200.0;

		// wst
		Deswst = 0.0;
		this->WSTmoveset(WBmotion, Deswst, this->Destime, MODE_ABSOLUTE);

		// rh
		this->Despos[0] = 0.45;
		this->Despos[1] = -0.35;
		this->Despos[2] = 0.35;

		this->RHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

		this->Temprv1[0] = 90*D2R;
		this->Temprv1[1] = 1;
		this->Temprv1[2] = 0;
		this->Temprv1[3] = 0;
		this->RV2QT(this->Temprv1, this->Tempth1);

		this->Temprv2[0] = -45*D2R;
		this->Temprv2[1] = 0;
		this->Temprv2[2] = 1;
		this->Temprv2[3] = 0;
		this->RV2QT(this->Temprv2, this->Tempth2);

		this->QTCross(this->Tempth1, this->Tempth2, this->Desth);

		this->RHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

		Deselb = RElbCal(Despos, Desth, Deswst);
		this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);


		RBwFTsensorNull(2, 53);
		RBwFTsensorNull(3, 54);

        joint->SetMoveJoint(RHAND, -125, 10, MODE_ABSOLUTE);
        joint->SetMoveJoint(LHAND, 0, 10, MODE_ABSOLUTE);


		break;

	case MPHASE_SW_GRASP_2:
		this->Goalcnt = 400;
		printf("\033[1;33m MPHASE_SW_GRASP_2 \033[0m\n");
		this->Destime = this->Goalcnt / 200.0;






//		bRHFTon = true;
//		bLHFTon = false;

		DockPoscal(this->SW_Tp, this->SW_Tw, FT_Distance + Palm_Distance + Dock_Distance, Dockpos);
		RHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);

		printf("%.3f %.3f %.3f \n", this->SW_Tp[0], this->SW_Tp[1], this->SW_Tp[2]);
		printf("%.3f %.3f %.3f \n", Dockpos[0], Dockpos[1], Dockpos[2]);

		this->Desth[0] = this->SW_Tw[0];
		this->Desth[1] = this->SW_Tw[1];
		this->Desth[2] = this->SW_Tw[2];
		this->Desth[3] = this->SW_Tw[3];

		this->RHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

		Deselb = RElbCal(Dockpos, this->SW_Tw, Deswst);
		this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);



		break;


	case MPHASE_SW_GRASP_3:
		this->Goalcnt = 400;
		printf("\033[1;33m MPHASE_SW_GRASP_3 \033[0m\n");
		this->Destime = this->Goalcnt / 200.0;



		// rh
		DockPoscal(this->SW_Tp, this->SW_Tw, FT_Distance + Palm_Distance, Dockpos);
		RHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);

		this->Desth[0] = this->SW_Tw[0];
		this->Desth[1] = this->SW_Tw[1];
		this->Desth[2] = this->SW_Tw[2];
		this->Desth[3] = this->SW_Tw[3];

		this->RHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

		Deselb = RElbCal(Despos, Desth, Deswst);
		this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);



		break;


	case MPHASE_SW_GRASP_4:
		this->Goalcnt = 600;
		printf("\033[1;33m MPHASE_SW_GRASP_4 \033[0m\n");
		this->Destime = this->Goalcnt / 200.0;

        joint->SetMoveJoint(RHAND, 125, 10, MODE_ABSOLUTE);


		break;


	case MPHASE_SW_GRASP_5:
		this->Goalcnt = 50;
		printf("\033[1;33m MPHASE_SW_GRASP_5 \033[0m\n");
		this->Destime = this->Goalcnt / 200.0;




		break;



		// ---------------------------------------------------------------------- //

	}
}


void CDebrisMotion::MotionGen_SW_Hold()
{
	EndMotionPhase = MPHASE_SW_HOLD_END;

	switch(this->MotionPhase){
	case MPHASE_SW_HOLD_READY:
		this->Goalcnt = 400;
		printf("\033[1;33m MPHASE_SW_HOLD_READY \033[0m\n");
		this->Destime = this->Goalcnt / 200.0;

		// wst
		Deswst = 0.0;
		this->WSTmoveset(WBmotion, Deswst, this->Destime, MODE_ABSOLUTE);

		printf("%.3f %.3f %.3f \n", WBmotion->pRH_3x1[0], WBmotion->pRH_3x1[1], WBmotion->pRH_3x1[2]);
		printf("%.2f \n", WBmotion->RElb_ang*R2D);

		// rh
		this->Despos[0] = WBmotion->pRH_3x1[0];
		this->Despos[1] = WBmotion->pRH_3x1[1];
		this->Despos[2] = WBmotion->pRH_3x1[2] + 0.1;

		this->RHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

		this->Desth[0] = WBmotion->qRH_4x1[0];
		this->Desth[1] = WBmotion->qRH_4x1[1];
		this->Desth[2] = WBmotion->qRH_4x1[2];
		this->Desth[3] = WBmotion->qRH_4x1[3];

		this->RHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

		Deselb = RElbCal(Despos, Desth, Deswst);
		printf("%.2f \n", Deselb*R2D);
		this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

		break;


	case MPHASE_SW_HOLD_1:
		this->Goalcnt = 600;
		printf("\033[1;33m MPHASE_SW_HOLD_1 \033[0m\n");
		this->Destime = this->Goalcnt / 200.0;



		// wst
		Deswst = 0.0;
		this->WSTmoveset(WBmotion, Deswst, this->Destime, MODE_ABSOLUTE);

		printf("%.3f %.3f %.3f \n", WBmotion->pRH_3x1[0], WBmotion->pRH_3x1[1], WBmotion->pRH_3x1[2]);
		printf("%.2f \n", WBmotion->RElb_ang*R2D);

		// rh
		this->Despos[0] = 0.45;
		this->Despos[1] = -0.35;
		this->Despos[2] = WBmotion->pRH_3x1[2];

		this->RHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

		this->Temprv1[0] = 90*D2R;
		this->Temprv1[1] = 1;
		this->Temprv1[2] = 0;
		this->Temprv1[3] = 0;
		RV2QT(this->Temprv1, this->Tempth1);

		this->Temprv2[0] = (-90 + 20)*D2R;
		this->Temprv2[1] = 0;
		this->Temprv2[2] = 1;
		this->Temprv2[3] = 0;
		this->RV2QT(this->Temprv2, this->Tempth2);

		QTCross(this->Tempth1, this->Tempth2, this->Desth);

		this->RHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

		Deselb = RElbCal(Despos, Desth, Deswst);
		printf("%.2f \n", Deselb*R2D);
		this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);


		break;

	case MPHASE_SW_HOLD_2:
		this->Goalcnt = 100;
		printf("\033[1;33m MPHASE_SW_HOLD_2 \033[0m\n");
		this->Destime = this->Goalcnt / 200.0;



		break;


	case MPHASE_SW_HOLD_3:
		this->Goalcnt = 10;
		printf("\033[1;33m MPHASE_SW_HOLD_3 \033[0m\n");
		this->Destime = this->Goalcnt / 200.0;



		break;


	case MPHASE_SW_HOLD_4:
		this->Goalcnt = 10;
		printf("\033[1;33m MPHASE_SW_HOLD_4 \033[0m\n");
		this->Destime = this->Goalcnt / 200.0;



		break;


	case MPHASE_SW_HOLD_5:
		this->Goalcnt = 10;
		printf("\033[1;33m MPHASE_SW_HOLD_5 \033[0m\n");
		this->Destime = this->Goalcnt / 200.0;



		break;



		// ---------------------------------------------------------------------- //

	}
}



void CDebrisMotion::MotionGen_SW_Back()
{
	EndMotionPhase = MPHASE_SW_BACK_END;

	switch(this->MotionPhase){
	case MPHASE_SW_BACK_READY:
        //this->Goalcnt = 500;
        this->Goalcnt = 5;
		printf("\033[1;33m MPHASE_SW_BACK_READY \033[0m\n");
		this->Destime = this->Goalcnt / 200.0;


       // joint->SetMoveJoint(RHAND, -125, 10, MODE_ABSOLUTE);




		break;


	case MPHASE_SW_BACK_1:
        //this->Goalcnt = 400;
        this->Goalcnt = 5;
		printf("\033[1;33m MPHASE_SW_BACK_1 \033[0m\n");
		this->Destime = this->Goalcnt / 200.0;


      //  joint->SetMoveJoint(RHAND, 125, 10, MODE_ABSOLUTE);


		break;

	case MPHASE_SW_BACK_2:
		this->Goalcnt = 800;
		printf("\033[1;33m MPHASE_SW_BACK_2 \033[0m\n");
		this->Destime = this->Goalcnt / 200.0;

        joint->SetMoveJoint(RHAND, 0, 10, MODE_ABSOLUTE);




		// wst
		Deswst = 0.0;
		this->WSTmoveset(WBmotion, Deswst, this->Destime, MODE_ABSOLUTE);

		// rh
		this->Despos[0] = 0.293;
		this->Despos[1] = -0.239;
		this->Despos[2] = 0.249;
		this->RHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

		this->Desrv[0] = 1.573;
		this->Desrv[1] = -0.005;
		this->Desrv[2] = -0.998;
		this->Desrv[3] = 0.061;
		this->RV2QT(this->Desrv, this->Desth);
		this->RHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

		this->RElbmoveset(WBmotion, -4.78*D2R, this->Destime, MODE_ABSOLUTE);


		break;


	case MPHASE_SW_BACK_3:
		this->Goalcnt = 40;
		printf("\033[1;33m MPHASE_SW_BACK_3 \033[0m\n");
		this->Destime = this->Goalcnt / 200.0;



		break;


	case MPHASE_SW_BACK_4:
		this->Goalcnt = 40;
		printf("\033[1;33m MPHASE_SW_BACK_4 \033[0m\n");
		this->Destime = this->Goalcnt / 200.0;



		break;


	case MPHASE_SW_BACK_5:
		this->Goalcnt = 40;
		printf("\033[1;33m MPHASE_SW_BACK_5 \033[0m\n");
		this->Destime = this->Goalcnt / 200.0;



		break;



		// ---------------------------------------------------------------------- //

	}
}




void CDebrisMotion::MotionGen_LeftSideStack()
{
    EndMotionPhase = MPHASE_STACKLEFTSIDE_END;

    switch(this->MotionPhase){
    case MPHASE_STACKLEFTSIDE_0:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE_STACKLEFTSIDE_0 \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

//        // wst
//        Deswst = -50.0;
//        this->WSTmoveset(WBmotion, Deswst, this->Destime, MODE_ABSOLUTE);

        PELZmoveset(WBmotion, 0.12, Destime, MODE_ABSOLUTE);

        break;

    case MPHASE_STACKLEFTSIDE_READY:
        this->Goalcnt = 800;
        printf("\033[1;33m MPHASE_STACKLEFTSIDE_READY \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

            printf("RH pos : %.3f %.3f %.3f %.3f \n", WBmotion->pRH_3x1[0], WBmotion->pRH_3x1[1], WBmotion->pRH_3x1[2], WBmotion->RElb_ang*R2D);
            printf("LH pos : %.3f %.3f %.3f %.3f \n", WBmotion->pLH_3x1[0], WBmotion->pLH_3x1[1], WBmotion->pLH_3x1[2], WBmotion->LElb_ang*R2D);


        // wst
        Deswst = 0.0;
        this->WSTmoveset(WBmotion, Deswst, this->Destime, MODE_ABSOLUTE);

        // rh
        this->Despos[0] = 0.55;
        this->Despos[1] = -0.3;
        this->Despos[2] = 0.45;

        this->RHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

        this->Desrv[0] = -45*D2R;
        this->Desrv[1] = 0;
        this->Desrv[2] = 1;
        this->Desrv[3] = 0;
        this->RV2QT(this->Desrv, this->Desth);

        this->RHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Despos, Desth, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        // lh
        this->Despos[0] = 0.55;
        this->Despos[1] = 0.3;
        this->Despos[2] = 0.45;

        this->LHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

        this->Desrv[0] = -45*D2R;
        this->Desrv[1] = 0;
        this->Desrv[2] = 1;
        this->Desrv[3] = 0;
        this->RV2QT(this->Desrv, this->Desth);

        this->LHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Despos, Desth, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);


        break;

    case MPHASE_STACKLEFTSIDE_WSTTURN90:
        this->Goalcnt = 700;
        printf("\033[1;33m MPHASE_STACKLEFTSIDE_WSTTURN \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        WSTmoveset(WBmotion, -90.0, Destime, MODE_ABSOLUTE);

        PELZmoveset(WBmotion, -0.0, Destime, MODE_ABSOLUTE);

        break;

    case MPHASE_STACKLEFTSIDE_RH:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE_STACKLEFTSIDE_RH \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        DockPoscal(DebS[2].Pos_r, DebS[2].Quat_r, Dock_Distance + Palm_Distance, Dockpos);
        RHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);
        RHOrimoveset(WBmotion, DebS[2].Quat_r, Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Dockpos, DebS[2].Quat_r, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        joint->SetMoveJoint(RHAND, -125, 10, MODE_ABSOLUTE);

        break;

    case MPHASE_STACKLEFTSIDE_LH:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE_STACKLEFTSIDE_LH \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        DockPoscal(DebS[3].Pos_r, DebS[3].Quat_r, Dock_Distance + Palm_Distance, Dockpos);
        LHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);
        LHOrimoveset(WBmotion, DebS[3].Quat_r, Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Dockpos, DebS[3].Quat_r, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        joint->SetMoveJoint(LHAND, -125, 10, MODE_ABSOLUTE);

        break;



    case MPHASE_STACKLEFTSIDE_FINOPEN:
        this->Goalcnt = 100;
        printf("\033[1;33m MPHASE_STACKLEFTSIDE_FINOPEN \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bEncCheckRH = true;
        bEncCheckLH = true;

        RBwFTsensorNull(2, 53);
        RBwFTsensorNull(3, 54);

        joint->SetMoveJoint(RHAND, -125, 10, MODE_ABSOLUTE);
        joint->SetMoveJoint(LHAND, -125, 10, MODE_ABSOLUTE);


        break;

    case MPHASE_STACKLEFTSIDE_RHDOCK:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE_STACKLEFTSIDE_RHDOCK \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bEncCheckRH = false;
        bEncCheckLH = false;

        bRHFTon = true;
        bLHFTon = false;

        DockPoscal(DebS[2].Pos_r, DebS[2].Quat_r, FT_Distance + Palm_Distance, Dockpos);
        RHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Dockpos, DebS[2].Quat_r, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;

    case MPHASE_STACKLEFTSIDE_LHDOCK:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE_STACKLEFTSIDE_LHDOCK \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bRHFTon = false;
        bLHFTon = true;

        DockPoscal(DebS[3].Pos_r, DebS[3].Quat_r, FT_Distance + Palm_Distance, Dockpos);
        LHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Dockpos, DebS[3].Quat_r, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;

    case MPHASE_STACKLEFTSIDE_RHGRASP:
        this->Goalcnt = 10;
        printf("\033[1;33m MPHASE_STACKLEFTSIDE_RHGRASP \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bRHFTon = false;
        bLHFTon = false;

        joint->SetMoveJoint(RHAND, 125, 10, MODE_ABSOLUTE);

        break;

    case MPHASE_STACKLEFTSIDE_LHGRASP:
        this->Goalcnt = 10;
        printf("\033[1;33m MPHASE_STACKLEFTSIDE_LHGRASP \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        joint->SetMoveJoint(LHAND, 125, 10, MODE_ABSOLUTE);

        break;

    case MPHASE_STACKLEFTSIDE_FINCLOSE:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE_STACKLEFTSIDE_FINCLOSE \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        joint->SetMoveJoint(RHAND, 125, 10, MODE_ABSOLUTE);
        joint->SetMoveJoint(LHAND, 125, 10, MODE_ABSOLUTE);


        break;

    case MPHASE_STACKLEFTSIDE_RHLIFT:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE_STACKLEFTSIDE_RHLIFT \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        DockPoscal(DebS[2].Pos_r, DebS[2].Quat_r, Lift_Distance + Palm_Distance, Dockpos);
        RHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Dockpos, DebS[2].Quat_r, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;

    case MPHASE_STACKLEFTSIDE_RHHOLD:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE_STACKLEFTSIDE_RHHOLD \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        // rh
        this->Despos[0] = 0.55;
        this->Despos[1] = -0.3;
        this->Despos[2] = 0.55;

        this->RHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

        this->Temprv1[0] = 70*D2R;
        this->Temprv1[1] = 1;
        this->Temprv1[2] = 0;
        this->Temprv1[3] = 0;
        this->RV2QT(this->Temprv1, this->Tempth1);

        this->Temprv2[0] = -80*D2R;
        this->Temprv2[1] = 0;
        this->Temprv2[2] = 1;
        this->Temprv2[3] = 0;
        this->RV2QT(this->Temprv2, this->Tempth2);

        QTCross(Tempth1, Tempth2, Desth);

        this->RHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Despos, Desth, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;

    case MPHASE_STACKLEFTSIDE_LHLIFT:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE_STACKLEFTSIDE_LHLIFT \033[0m\n");
        //bEncCheck = true;
        this->Destime = this->Goalcnt / 200.0;

        DockPoscal(DebS[3].Pos_r, DebS[3].Quat_r, Lift_Distance + Palm_Distance, Dockpos);
        LHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Dockpos, DebS[3].Quat_r, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;

    case MPHASE_STACKLEFTSIDE_LHHOLD:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE_STACKLEFTSIDE_LHHOLD \033[0m\n");
        //bEncCheck = true;
        this->Destime = this->Goalcnt / 200.0;

        // lh
        this->Despos[0] = 0.55;
        this->Despos[1] = 0.3;
        this->Despos[2] = 0.55;

        this->LHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

        this->Temprv1[0] = -70*D2R;
        this->Temprv1[1] = 1;
        this->Temprv1[2] = 0;
        this->Temprv1[3] = 0;
        this->RV2QT(this->Temprv1, this->Tempth1);

        this->Temprv2[0] = -80*D2R;
        this->Temprv2[1] = 0;
        this->Temprv2[2] = 1;
        this->Temprv2[3] = 0;
        this->RV2QT(this->Temprv2, this->Tempth2);

        QTCross(Tempth1, Tempth2, Desth);

        this->LHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Despos, Desth, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;






    case MPHASE_STACKLEFTSIDE_WSTTURN_90:
        this->Goalcnt = 1000;
        printf("\033[1;33m MPHASE_STACKLEFTSIDE_WSTTURN \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        //WBIKframe(WBmotion, WBmode_RULU);

        WSTmoveset(WBmotion, 90.0, Destime, MODE_ABSOLUTE);

        printf("cur comx %.3f \n", WBmotion->pCOM_2x1[0]);
        //COMmoveset(WBmotion, 0.00, 0, Destime, MODE_ABSOLUTE);

        PELZmoveset(WBmotion, -0.12, Destime, MODE_ABSOLUTE);

        break;






    case MPHASE_STACKLEFTSIDE_RHPUT:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE_STACKLEFTSIDE_RHPUT \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        DockPoscal(DebG[2].Pos_r, DebG[2].Quat_r, Dock_Distance + Palm_Distance, Dockpos);
        RHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);
        RHOrimoveset(WBmotion, DebG[2].Quat_r, Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Dockpos, DebG[2].Quat_r, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;


    case MPHASE_STACKLEFTSIDE_RHNULL:
        this->Goalcnt = 10;
        printf("\033[1;33m MPHASE_STACKLEFTSIDE_RHNULL \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        RBwFTsensorNull(2, 53);
        //RBwFTsensorNull(3, 54);

        break;


    case MPHASE_STACKLEFTSIDE_RHDOWN:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE_STACKLEFTSIDE_RHDOWN \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bRHFTon = true;

        DockPoscal(DebG[2].Pos_r, DebG[2].Quat_r, FT_Distance + Palm_Distance, Dockpos);
        RHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Dockpos, DebG[2].Quat_r, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);


        break;

    case MPHASE_STACKLEFTSIDE_RHOFF:
        this->Goalcnt = 100;
        printf("\033[1;33m MPHASE_STACKLEFTSIDE_RHOFF \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bRHFTon = false;

        bEncCheckRH = true;

        joint->SetMoveJoint(RHAND, -125, 10, MODE_ABSOLUTE);

        break;

    case MPHASE_STACKLEFTSIDE_RHUP:
        this->Goalcnt = 300;
        printf("\033[1;33m MPHASE_STACKLEFTSIDE_RHUP \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bEncCheckRH = false;

        DockPoscal(DebG[2].Pos_r, DebG[2].Quat_r, Dock_Distance + Palm_Distance, Dockpos);
        RHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);
        RHOrimoveset(WBmotion, DebG[2].Quat_r, Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Dockpos, DebG[2].Quat_r, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;

    case MPHASE_STACKLEFTSIDE_RHON:
        this->Goalcnt = 100;
        printf("\033[1;33m MPHASE_STACKLEFTSIDE_RHON \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        joint->SetMoveJoint(RHAND, 125, 10, MODE_ABSOLUTE);

        break;


    case MPHASE_STACKLEFTSIDE_LHPUT:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE_STACKLEFTSIDE_LHPUT \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        DockPoscal(DebG[3].Pos_r, DebG[3].Quat_r, Dock_Distance + Palm_Distance, Dockpos);
        LHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);
        LHOrimoveset(WBmotion, DebG[3].Quat_r, Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Dockpos, DebG[3].Quat_r, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;


    case MPHASE_STACKLEFTSIDE_LHNULL:
        this->Goalcnt = 10;
        printf("\033[1;33m MPHASE_STACKLEFTSIDE_LHNULL \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        //RBwFTsensorNull(2, 53);
        RBwFTsensorNull(3, 54);

        break;


    case MPHASE_STACKLEFTSIDE_LHDOWN:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE_STACKLEFTSIDE_LHDOWN \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bLHFTon = true;

        DockPoscal(DebG[3].Pos_r, DebG[3].Quat_r, FT_Distance + Palm_Distance, Dockpos);
        LHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Dockpos, DebG[3].Quat_r, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);


        break;

    case MPHASE_STACKLEFTSIDE_LHOFF:
        this->Goalcnt = 500;
        printf("\033[1;33m MPHASE_STACKLEFTSIDE_LHOFF \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bLHFTon = false;

        bEncCheckLH = true;

        joint->SetMoveJoint(LHAND, -125, 10, MODE_ABSOLUTE);

        break;

    case MPHASE_STACKLEFTSIDE_LHUP:
        this->Goalcnt = 300;
        printf("\033[1;33m MPHASE_STACKLEFTSIDE_LHUP \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bEncCheckLH = false;

        DockPoscal(DebG[3].Pos_r, DebG[3].Quat_r, Dock_Distance + Palm_Distance, Dockpos);
        LHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);
        LHOrimoveset(WBmotion, DebG[3].Quat_r, Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Dockpos, DebG[3].Quat_r, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;

    case MPHASE_STACKLEFTSIDE_LHON:
        this->Goalcnt = 100;
        printf("\033[1;33m MPHASE_STACKLEFTSIDE_LHON \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        joint->SetMoveJoint(LHAND, 125, 10, MODE_ABSOLUTE);

        break;

    case MPHASE_STACKLEFTSIDE_HANDREADY:
        this->Goalcnt = 500;
        printf("\033[1;33m MPHASE_STACKLEFTSIDE_HANDREADY \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        // rh
        this->Despos[0] = 0.55;
        this->Despos[1] = -0.3;
        this->Despos[2] = 0.45;

        this->RHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

        this->Desrv[0] = -45*D2R;
        this->Desrv[1] = 0;
        this->Desrv[2] = 1;
        this->Desrv[3] = 0;
        this->RV2QT(this->Desrv, this->Desth);

        this->RHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Despos, Desth, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        // lh
        this->Despos[0] = 0.55;
        this->Despos[1] = 0.3;
        this->Despos[2] = 0.45;

        this->LHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

        this->Desrv[0] = -45*D2R;
        this->Desrv[1] = 0;
        this->Desrv[2] = 1;
        this->Desrv[3] = 0;
        this->RV2QT(this->Desrv, this->Desth);

        this->LHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Despos, Desth, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        PELZmoveset(WBmotion, 0.12, Destime, MODE_ABSOLUTE);

        break;

    case MPHASE_STACKLEFTSIDE_WSTTURN0:
        this->Goalcnt = 500;
        printf("\033[1;33m MPHASE_STACKLEFTSIDE_WSTTURN0 \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        WSTmoveset(WBmotion, 0.0, Destime, MODE_ABSOLUTE);

        this->Despos[0] = WBmotion->pRH_3x1[0];
        this->Despos[1] = WBmotion->pRH_3x1[1];
        this->Despos[2] = WBmotion->pRH_3x1[2];

        this->Desrv[0] = -80*D2R;
        this->Desrv[1] = 0;
        this->Desrv[2] = 1;
        this->Desrv[3] = 0;
        this->RV2QT(this->Desrv, this->Desth);

        this->RHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Despos, Desth, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);


        this->Despos[0] = WBmotion->pLH_3x1[0];
        this->Despos[1] = WBmotion->pLH_3x1[1];
        this->Despos[2] = WBmotion->pLH_3x1[2];

        this->Desrv[0] = -80*D2R;
        this->Desrv[1] = 0;
        this->Desrv[2] = 1;
        this->Desrv[3] = 0;
        this->RV2QT(this->Desrv, this->Desth);

        this->LHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Despos, Desth, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;


    case MPHASE_STACKLEFTSIDE_HOME:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE_STACKLEFTSIDE_HOME \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        // rh
        this->Despos[0] = 0.291;
        this->Despos[1] = -0.241;
        this->Despos[2] = 0.221;

        this->RHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

        this->Desrv[0] = -90*D2R;
        this->Desrv[1] = 0;
        this->Desrv[2] = 1;
        this->Desrv[3] = 0;
        this->RV2QT(this->Desrv, this->Desth);

        this->RHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        this->RElbmoveset(WBmotion, -4.873, this->Destime, MODE_ABSOLUTE);

        // lh
        this->Despos[0] = 0.291;
        this->Despos[1] = 0.241;
        this->Despos[2] = 0.221;

        this->LHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

        this->Desrv[0] = -90*D2R;
        this->Desrv[1] = 0;
        this->Desrv[2] = 1;
        this->Desrv[3] = 0;
        this->RV2QT(this->Desrv, this->Desth);

        this->LHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        this->LElbmoveset(WBmotion, 4.873, this->Destime, MODE_ABSOLUTE);


        joint->SetMoveJoint(RHAND, 0, 10, MODE_ABSOLUTE);
        joint->SetMoveJoint(LHAND, 0, 10, MODE_ABSOLUTE);

        break;


        // ---------------------------------------------------------------------- //










    case MPHASE1_STACKLEFTSIDE_0:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE1_STACKLEFTSIDE_0 \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        // wst
//        Deswst = -50.0;
//        this->WSTmoveset(WBmotion, Deswst, this->Destime, MODE_ABSOLUTE);

        PELZmoveset(WBmotion, 0.12, Destime, MODE_ABSOLUTE);

        break;

    case MPHASE1_STACKLEFTSIDE_READY:
        this->Goalcnt = 800;
        printf("\033[1;33m MPHASE1_STACKLEFTSIDE_READY \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

            printf("RH pos : %.3f %.3f %.3f %.3f \n", WBmotion->pRH_3x1[0], WBmotion->pRH_3x1[1], WBmotion->pRH_3x1[2], WBmotion->RElb_ang*R2D);
            printf("LH pos : %.3f %.3f %.3f %.3f \n", WBmotion->pLH_3x1[0], WBmotion->pLH_3x1[1], WBmotion->pLH_3x1[2], WBmotion->LElb_ang*R2D);


        // wst
        Deswst = -0.0;
        this->WSTmoveset(WBmotion, Deswst, this->Destime, MODE_ABSOLUTE);

        // rh
        this->Despos[0] = 0.55;
        this->Despos[1] = -0.3;
        this->Despos[2] = 0.45;

        this->RHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

        this->Desrv[0] = -45*D2R;
        this->Desrv[1] = 0;
        this->Desrv[2] = 1;
        this->Desrv[3] = 0;
        this->RV2QT(this->Desrv, this->Desth);

        this->RHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Despos, Desth, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        // lh
        this->Despos[0] = 0.55;
        this->Despos[1] = 0.3;
        this->Despos[2] = 0.45;

        this->LHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

        this->Desrv[0] = -45*D2R;
        this->Desrv[1] = 0;
        this->Desrv[2] = 1;
        this->Desrv[3] = 0;
        this->RV2QT(this->Desrv, this->Desth);

        this->LHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Despos, Desth, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;

    case MPHASE1_STACKLEFTSIDE_WSTTURN90:
        this->Goalcnt = 700;
        printf("\033[1;33m MPHASE1_STACKLEFTSIDE_WSTTURN \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        WSTmoveset(WBmotion, -90.0, Destime, MODE_ABSOLUTE);
        PELZmoveset(WBmotion, 0.0, Destime, MODE_ABSOLUTE);

        break;

    case MPHASE1_STACKLEFTSIDE_RH:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE1_STACKLEFTSIDE_RH \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        DockPoscal(DebS[0].Pos_r, DebS[0].Quat_r, Dock_Distance + Palm_Distance, Dockpos);
        RHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);
        RHOrimoveset(WBmotion, DebS[0].Quat_r, Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Dockpos, DebS[0].Quat_r, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        joint->SetMoveJoint(RHAND, -125, 10, MODE_ABSOLUTE);

        break;

    case MPHASE1_STACKLEFTSIDE_LH:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE1_STACKLEFTSIDE_LH \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        DockPoscal(DebS[1].Pos_r, DebS[1].Quat_r, Dock_Distance + Palm_Distance, Dockpos);
        LHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);
        LHOrimoveset(WBmotion, DebS[1].Quat_r, Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Dockpos, DebS[1].Quat_r, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        joint->SetMoveJoint(LHAND, -125, 10, MODE_ABSOLUTE);

        break;



    case MPHASE1_STACKLEFTSIDE_FINOPEN:
        this->Goalcnt = 100;
        printf("\033[1;33m MPHASE1_STACKLEFTSIDE_FINOPEN \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bEncCheckRH = true;
        bEncCheckLH = true;

        RBwFTsensorNull(2, 53);
        RBwFTsensorNull(3, 54);

        joint->SetMoveJoint(RHAND, -125, 10, MODE_ABSOLUTE);
        joint->SetMoveJoint(LHAND, -125, 10, MODE_ABSOLUTE);


        break;

    case MPHASE1_STACKLEFTSIDE_RHDOCK:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE1_STACKLEFTSIDE_RHDOCK \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bEncCheckRH = false;
        bEncCheckLH = false;

        bRHFTon = true;
        bLHFTon = false;

        DockPoscal(DebS[0].Pos_r, DebS[0].Quat_r, FT_Distance + Palm_Distance, Dockpos);
        RHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Dockpos, DebS[0].Quat_r, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;

    case MPHASE1_STACKLEFTSIDE_LHDOCK:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE1_STACKLEFTSIDE_LHDOCK \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bRHFTon = false;
        bLHFTon = true;

        DockPoscal(DebS[1].Pos_r, DebS[1].Quat_r, FT_Distance + Palm_Distance, Dockpos);
        LHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Dockpos, DebS[1].Quat_r, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;

    case MPHASE1_STACKLEFTSIDE_RHGRASP:
        this->Goalcnt = 10;
        printf("\033[1;33m MPHASE1_STACKLEFTSIDE_RHGRASP \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bRHFTon = false;
        bLHFTon = false;

        joint->SetMoveJoint(RHAND, 125, 10, MODE_ABSOLUTE);

        break;

    case MPHASE1_STACKLEFTSIDE_LHGRASP:
        this->Goalcnt = 10;
        printf("\033[1;33m MPHASE1_STACKLEFTSIDE_LHGRASP \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        joint->SetMoveJoint(LHAND, 125, 10, MODE_ABSOLUTE);

        break;

    case MPHASE1_STACKLEFTSIDE_FINCLOSE:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE1_STACKLEFTSIDE_FINCLOSE \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        joint->SetMoveJoint(RHAND, 125, 10, MODE_ABSOLUTE);
        joint->SetMoveJoint(LHAND, 125, 10, MODE_ABSOLUTE);


        break;

    case MPHASE1_STACKLEFTSIDE_RHLIFT:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE1_STACKLEFTSIDE_RHLIFT \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        DockPoscal(DebS[0].Pos_r, DebS[0].Quat_r, Lift_Distance + Palm_Distance, Dockpos);
        RHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Dockpos, DebS[0].Quat_r, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;

    case MPHASE1_STACKLEFTSIDE_RHHOLD:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE1_STACKLEFTSIDE_RHHOLD \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        // rh
        this->Despos[0] = 0.55;
        this->Despos[1] = -0.3;
        this->Despos[2] = 0.55;

        this->RHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

        this->Temprv1[0] = 70*D2R;
        this->Temprv1[1] = 1;
        this->Temprv1[2] = 0;
        this->Temprv1[3] = 0;
        this->RV2QT(this->Temprv1, this->Tempth1);

        this->Temprv2[0] = -80*D2R;
        this->Temprv2[1] = 0;
        this->Temprv2[2] = 1;
        this->Temprv2[3] = 0;
        this->RV2QT(this->Temprv2, this->Tempth2);

        QTCross(Tempth1, Tempth2, Desth);

        this->RHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Despos, Desth, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;

    case MPHASE1_STACKLEFTSIDE_LHLIFT:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE1_STACKLEFTSIDE_LHLIFT \033[0m\n");
        //bEncCheck = true;
        this->Destime = this->Goalcnt / 200.0;

        DockPoscal(DebS[1].Pos_r, DebS[1].Quat_r, Lift_Distance + Palm_Distance, Dockpos);
        LHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Dockpos, DebS[1].Quat_r, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;

    case MPHASE1_STACKLEFTSIDE_LHHOLD:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE1_STACKLEFTSIDE_LHHOLD \033[0m\n");
        //bEncCheck = true;
        this->Destime = this->Goalcnt / 200.0;

        // lh
        this->Despos[0] = 0.55;
        this->Despos[1] = 0.3;
        this->Despos[2] = 0.55;

        this->LHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

        this->Temprv1[0] = -70*D2R;
        this->Temprv1[1] = 1;
        this->Temprv1[2] = 0;
        this->Temprv1[3] = 0;
        this->RV2QT(this->Temprv1, this->Tempth1);

        this->Temprv2[0] = -80*D2R;
        this->Temprv2[1] = 0;
        this->Temprv2[2] = 1;
        this->Temprv2[3] = 0;
        this->RV2QT(this->Temprv2, this->Tempth2);

        QTCross(Tempth1, Tempth2, Desth);

        this->LHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Despos, Desth, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        PELZmoveset(WBmotion, -0.00, Destime, MODE_ABSOLUTE);

        break;






    case MPHASE1_STACKLEFTSIDE_WSTTURN_90:
        this->Goalcnt = 1000;
        printf("\033[1;33m MPHASE1_STACKLEFTSIDE_WSTTURN \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        //WBIKframe(WBmotion, WBmode_RULU);

        WSTmoveset(WBmotion, 90.0, Destime, MODE_ABSOLUTE);

        printf("cur comx %.3f \n", WBmotion->pCOM_2x1[0]);
        //COMmoveset(WBmotion, 0.00, 0, Destime, MODE_ABSOLUTE);

        break;






    case MPHASE1_STACKLEFTSIDE_RHPUT:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE1_STACKLEFTSIDE_RHPUT \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        DockPoscal(DebG[0].Pos_r, DebG[0].Quat_r, Dock_Distance + Palm_Distance, Dockpos);
        RHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);
        RHOrimoveset(WBmotion, DebG[0].Quat_r, Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Dockpos, DebG[0].Quat_r, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;


    case MPHASE1_STACKLEFTSIDE_RHNULL:
        this->Goalcnt = 10;
        printf("\033[1;33m MPHASE1_STACKLEFTSIDE_RHNULL \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        RBwFTsensorNull(2, 53);
        //RBwFTsensorNull(3, 54);

        break;


    case MPHASE1_STACKLEFTSIDE_RHDOWN:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE1_STACKLEFTSIDE_RHDOWN \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bRHFTon = true;

        DockPoscal(DebG[0].Pos_r, DebG[0].Quat_r, FT_Distance + Palm_Distance, Dockpos);
        RHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Dockpos, DebG[0].Quat_r, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);


        break;

    case MPHASE1_STACKLEFTSIDE_RHOFF:
        this->Goalcnt = 100;
        printf("\033[1;33m MPHASE1_STACKLEFTSIDE_RHOFF \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bRHFTon = false;

        bEncCheckRH = true;

        joint->SetMoveJoint(RHAND, -125, 10, MODE_ABSOLUTE);

        break;

    case MPHASE1_STACKLEFTSIDE_RHUP:
        this->Goalcnt = 300;
        printf("\033[1;33m MPHASE1_STACKLEFTSIDE_RHUP \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bEncCheckRH = false;

        DockPoscal(DebG[0].Pos_r, DebG[0].Quat_r, Dock_Distance + Palm_Distance, Dockpos);
        RHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);
        RHOrimoveset(WBmotion, DebG[0].Quat_r, Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Dockpos, DebG[0].Quat_r, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;

    case MPHASE1_STACKLEFTSIDE_RHON:
        this->Goalcnt = 100;
        printf("\033[1;33m MPHASE1_STACKLEFTSIDE_RHON \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        joint->SetMoveJoint(RHAND, 125, 10, MODE_ABSOLUTE);

        break;


    case MPHASE1_STACKLEFTSIDE_LHPUT:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE1_STACKLEFTSIDE_LHPUT \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        DockPoscal(DebG[1].Pos_r, DebG[1].Quat_r, Dock_Distance + Palm_Distance, Dockpos);
        LHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);
        LHOrimoveset(WBmotion, DebG[1].Quat_r, Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Dockpos, DebG[1].Quat_r, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;


    case MPHASE1_STACKLEFTSIDE_LHNULL:
        this->Goalcnt = 10;
        printf("\033[1;33m MPHASE1_STACKLEFTSIDE_LHNULL \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        //RBwFTsensorNull(2, 53);
        RBwFTsensorNull(3, 54);

        break;


    case MPHASE1_STACKLEFTSIDE_LHDOWN:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE1_STACKLEFTSIDE_LHDOWN \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bLHFTon = true;

        DockPoscal(DebG[1].Pos_r, DebG[1].Quat_r, FT_Distance + Palm_Distance, Dockpos);
        LHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Dockpos, DebG[1].Quat_r, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);


        break;

    case MPHASE1_STACKLEFTSIDE_LHOFF:
        this->Goalcnt = 500;
        printf("\033[1;33m MPHASE1_STACKLEFTSIDE_LHOFF \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bLHFTon = false;

        bEncCheckLH = true;

        joint->SetMoveJoint(LHAND, -125, 10, MODE_ABSOLUTE);

        break;

    case MPHASE1_STACKLEFTSIDE_LHUP:
        this->Goalcnt = 300;
        printf("\033[1;33m MPHASE1_STACKLEFTSIDE_LHUP \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bEncCheckLH = false;

        DockPoscal(DebG[1].Pos_r, DebG[1].Quat_r, Dock_Distance + Palm_Distance, Dockpos);
        LHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);
        LHOrimoveset(WBmotion, DebG[1].Quat_r, Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Dockpos, DebG[1].Quat_r, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;

    case MPHASE1_STACKLEFTSIDE_LHON:
        this->Goalcnt = 100;
        printf("\033[1;33m MPHASE1_STACKLEFTSIDE_LHON \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        joint->SetMoveJoint(LHAND, 125, 10, MODE_ABSOLUTE);

        break;

    case MPHASE1_STACKLEFTSIDE_HANDREADY:
        this->Goalcnt = 500;
        printf("\033[1;33m MPHASE1_STACKLEFTSIDE_HANDREADY \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        // rh
        this->Despos[0] = 0.55;
        this->Despos[1] = -0.3;
        this->Despos[2] = 0.45;

        this->RHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

        this->Desrv[0] = -45*D2R;
        this->Desrv[1] = 0;
        this->Desrv[2] = 1;
        this->Desrv[3] = 0;
        this->RV2QT(this->Desrv, this->Desth);

        this->RHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Despos, Desth, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        // lh
        this->Despos[0] = 0.55;
        this->Despos[1] = 0.3;
        this->Despos[2] = 0.45;

        this->LHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

        this->Desrv[0] = -45*D2R;
        this->Desrv[1] = 0;
        this->Desrv[2] = 1;
        this->Desrv[3] = 0;
        this->RV2QT(this->Desrv, this->Desth);

        this->LHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Despos, Desth, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;

    case MPHASE1_STACKLEFTSIDE_WSTTURN0:
        this->Goalcnt = 500;
        printf("\033[1;33m MPHASE1_STACKLEFTSIDE_WSTTURN0 \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        WSTmoveset(WBmotion, 0.0, Destime, MODE_ABSOLUTE);

        this->Despos[0] = WBmotion->pRH_3x1[0];
        this->Despos[1] = WBmotion->pRH_3x1[1];
        this->Despos[2] = WBmotion->pRH_3x1[2];

        this->Desrv[0] = -80*D2R;
        this->Desrv[1] = 0;
        this->Desrv[2] = 1;
        this->Desrv[3] = 0;
        this->RV2QT(this->Desrv, this->Desth);

        this->RHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Despos, Desth, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);


        this->Despos[0] = WBmotion->pLH_3x1[0];
        this->Despos[1] = WBmotion->pLH_3x1[1];
        this->Despos[2] = WBmotion->pLH_3x1[2];

        this->Desrv[0] = -80*D2R;
        this->Desrv[1] = 0;
        this->Desrv[2] = 1;
        this->Desrv[3] = 0;
        this->RV2QT(this->Desrv, this->Desth);

        this->LHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Despos, Desth, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;


    case MPHASE1_STACKLEFTSIDE_HOME:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE1_STACKLEFTSIDE_HOME \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        // rh
        this->Despos[0] = 0.291;
        this->Despos[1] = -0.241;
        this->Despos[2] = 0.221;

        this->RHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

        this->Desrv[0] = -90*D2R;
        this->Desrv[1] = 0;
        this->Desrv[2] = 1;
        this->Desrv[3] = 0;
        this->RV2QT(this->Desrv, this->Desth);

        this->RHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        this->RElbmoveset(WBmotion, -4.873, this->Destime, MODE_ABSOLUTE);

        // lh
        this->Despos[0] = 0.291;
        this->Despos[1] = 0.241;
        this->Despos[2] = 0.221;

        this->LHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

        this->Desrv[0] = -90*D2R;
        this->Desrv[1] = 0;
        this->Desrv[2] = 1;
        this->Desrv[3] = 0;
        this->RV2QT(this->Desrv, this->Desth);

        this->LHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        this->LElbmoveset(WBmotion, 4.873, this->Destime, MODE_ABSOLUTE);


        joint->SetMoveJoint(RHAND, 0, 10, MODE_ABSOLUTE);
        joint->SetMoveJoint(LHAND, 0, 10, MODE_ABSOLUTE);

        break;

    }
}


// ========================================================================= //


void CDebrisMotion::MotionGen_RightSidePickup()
{
	EndMotionPhase = MPHASE_PICKRIGHTSIDE_END;

	switch(this->MotionPhase){
    case MPHASE_PICKRIGHTSIDE_0:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE_PICKRIGHTSIDE_0 \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

//        // wst
//        Deswst = -50.0;
//        this->WSTmoveset(WBmotion, Deswst, this->Destime, MODE_ABSOLUTE);

        PELZmoveset(WBmotion, 0.12, Destime, MODE_ABSOLUTE);

        break;

	case MPHASE_PICKRIGHTSIDE_READY:
        this->Goalcnt = 800;
		printf("\033[1;33m MPHASE_PICKRIGHTSIDE_READY \033[0m\n");
		this->Destime = this->Goalcnt / 200.0;

            printf("RH pos : %.3f %.3f %.3f %.3f \n", WBmotion->pRH_3x1[0], WBmotion->pRH_3x1[1], WBmotion->pRH_3x1[2], WBmotion->RElb_ang*R2D);
            printf("LH pos : %.3f %.3f %.3f %.3f \n", WBmotion->pLH_3x1[0], WBmotion->pLH_3x1[1], WBmotion->pLH_3x1[2], WBmotion->LElb_ang*R2D);


		// wst
        Deswst = 0.0;
		this->WSTmoveset(WBmotion, Deswst, this->Destime, MODE_ABSOLUTE);

		// rh
		this->Despos[0] = 0.55;
		this->Despos[1] = -0.3;
		this->Despos[2] = 0.45;

		this->RHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

		this->Desrv[0] = -45*D2R;
		this->Desrv[1] = 0;
		this->Desrv[2] = 1;
		this->Desrv[3] = 0;
		this->RV2QT(this->Desrv, this->Desth);

		this->RHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

		Deselb = RElbCal(Despos, Desth, Deswst);
		this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

		// lh
		this->Despos[0] = 0.55;
		this->Despos[1] = 0.3;
		this->Despos[2] = 0.45;

		this->LHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

		this->Desrv[0] = -45*D2R;
		this->Desrv[1] = 0;
		this->Desrv[2] = 1;
		this->Desrv[3] = 0;
		this->RV2QT(this->Desrv, this->Desth);

		this->LHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

		Deselb = LElbCal(Despos, Desth, Deswst);
		this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);


		break;

    case MPHASE_PICKRIGHTSIDE_WSTTURN90:
        this->Goalcnt = 700;
        printf("\033[1;33m MPHASE_PICKRIGHTSIDE_WSTTURN \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        WSTmoveset(WBmotion, 90.0, Destime, MODE_ABSOLUTE);

        PELZmoveset(WBmotion, -0.0, Destime, MODE_ABSOLUTE);

        break;

	case MPHASE_PICKRIGHTSIDE_RH:
		this->Goalcnt = 400;
        printf("\033[1;33m MPHASE_PICKRIGHTSIDE_RH \033[0m\n");
		this->Destime = this->Goalcnt / 200.0;

		DockPoscal(DebG[0].Pos_r, DebG[0].Quat_r, Dock_Distance + Palm_Distance, Dockpos);
		RHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);
		RHOrimoveset(WBmotion, DebG[0].Quat_r, Destime, MODE_ABSOLUTE);

		Deselb = RElbCal(Dockpos, DebG[0].Quat_r, Deswst);
		this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        joint->SetMoveJoint(RHAND, -125, 10, MODE_ABSOLUTE);

		break;

	case MPHASE_PICKRIGHTSIDE_LH:
		this->Goalcnt = 400;
        printf("\033[1;33m MPHASE_PICKRIGHTSIDE_LH \033[0m\n");
		this->Destime = this->Goalcnt / 200.0;

		DockPoscal(DebG[1].Pos_r, DebG[1].Quat_r, Dock_Distance + Palm_Distance, Dockpos);
		LHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);
		LHOrimoveset(WBmotion, DebG[1].Quat_r, Destime, MODE_ABSOLUTE);

		Deselb = LElbCal(Dockpos, DebG[1].Quat_r, Deswst);
		this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        joint->SetMoveJoint(LHAND, -125, 10, MODE_ABSOLUTE);

		break;



	case MPHASE_PICKRIGHTSIDE_FINOPEN:
		this->Goalcnt = 100;
        printf("\033[1;33m MPHASE_PICKRIGHTSIDE_FINOPEN \033[0m\n");
		this->Destime = this->Goalcnt / 200.0;

        bEncCheckRH = true;
        bEncCheckLH = true;

		RBwFTsensorNull(2, 53);
		RBwFTsensorNull(3, 54);

        joint->SetMoveJoint(RHAND, -125, 10, MODE_ABSOLUTE);
        joint->SetMoveJoint(LHAND, -125, 10, MODE_ABSOLUTE);


		break;

	case MPHASE_PICKRIGHTSIDE_RHDOCK:
		this->Goalcnt = 400;
		printf("\033[1;33m MPHASE_PICKRIGHTSIDE_RHDOCK \033[0m\n");
		this->Destime = this->Goalcnt / 200.0;

        bEncCheckRH = false;
        bEncCheckLH = false;

		bRHFTon = true;
		bLHFTon = false;

		DockPoscal(DebG[0].Pos_r, DebG[0].Quat_r, FT_Distance + Palm_Distance, Dockpos);
		RHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);

		Deselb = RElbCal(Dockpos, DebG[0].Quat_r, Deswst);
		this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

		break;

	case MPHASE_PICKRIGHTSIDE_LHDOCK:
		this->Goalcnt = 400;
		printf("\033[1;33m MPHASE_PICKRIGHTSIDE_LHDOCK \033[0m\n");
		this->Destime = this->Goalcnt / 200.0;

		bRHFTon = false;
		bLHFTon = true;

		DockPoscal(DebG[1].Pos_r, DebG[1].Quat_r, FT_Distance + Palm_Distance, Dockpos);
		LHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);

		Deselb = LElbCal(Dockpos, DebG[1].Quat_r, Deswst);
		this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

		break;

	case MPHASE_PICKRIGHTSIDE_RHGRASP:
        this->Goalcnt = 10;
		printf("\033[1;33m MPHASE_PICKRIGHTSIDE_RHGRASP \033[0m\n");
		this->Destime = this->Goalcnt / 200.0;

		bRHFTon = false;
		bLHFTon = false;

        joint->SetMoveJoint(RHAND, 125, 10, MODE_ABSOLUTE);

		break;

	case MPHASE_PICKRIGHTSIDE_LHGRASP:
        this->Goalcnt = 10;
		printf("\033[1;33m MPHASE_PICKRIGHTSIDE_LHGRASP \033[0m\n");
		this->Destime = this->Goalcnt / 200.0;

        joint->SetMoveJoint(LHAND, 125, 10, MODE_ABSOLUTE);

		break;

	case MPHASE_PICKRIGHTSIDE_FINCLOSE:
        this->Goalcnt = 400;
		printf("\033[1;33m MPHASE_PICKRIGHTSIDE_FINCLOSE \033[0m\n");
		this->Destime = this->Goalcnt / 200.0;

        joint->SetMoveJoint(RHAND, 125, 10, MODE_ABSOLUTE);
        joint->SetMoveJoint(LHAND, 125, 10, MODE_ABSOLUTE);


		break;

	case MPHASE_PICKRIGHTSIDE_RHLIFT:
		this->Goalcnt = 400;
		printf("\033[1;33m MPHASE_PICKRIGHTSIDE_RHLIFT \033[0m\n");
		this->Destime = this->Goalcnt / 200.0;

		DockPoscal(DebG[0].Pos_r, DebG[0].Quat_r, Lift_Distance + Palm_Distance, Dockpos);
		RHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);

		Deselb = RElbCal(Dockpos, DebG[0].Quat_r, Deswst);
		this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

		break;

	case MPHASE_PICKRIGHTSIDE_RHHOLD:
		this->Goalcnt = 400;
		printf("\033[1;33m MPHASE_PICKRIGHTSIDE_RHHOLD \033[0m\n");
		this->Destime = this->Goalcnt / 200.0;

		// rh
		this->Despos[0] = 0.55;
		this->Despos[1] = -0.3;
		this->Despos[2] = 0.55;

		this->RHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

		this->Temprv1[0] = 70*D2R;
		this->Temprv1[1] = 1;
		this->Temprv1[2] = 0;
		this->Temprv1[3] = 0;
		this->RV2QT(this->Temprv1, this->Tempth1);

		this->Temprv2[0] = -80*D2R;
		this->Temprv2[1] = 0;
		this->Temprv2[2] = 1;
		this->Temprv2[3] = 0;
		this->RV2QT(this->Temprv2, this->Tempth2);

		QTCross(Tempth1, Tempth2, Desth);

		this->RHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

		Deselb = RElbCal(Despos, Desth, Deswst);
		this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

		break;

	case MPHASE_PICKRIGHTSIDE_LHLIFT:
		this->Goalcnt = 400;
		printf("\033[1;33m MPHASE_PICKRIGHTSIDE_LHLIFT \033[0m\n");
		//bEncCheck = true;
		this->Destime = this->Goalcnt / 200.0;

		DockPoscal(DebG[1].Pos_r, DebG[1].Quat_r, Lift_Distance + Palm_Distance, Dockpos);
		LHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);

		Deselb = LElbCal(Dockpos, DebG[1].Quat_r, Deswst);
		this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

		break;

	case MPHASE_PICKRIGHTSIDE_LHHOLD:
		this->Goalcnt = 400;
		printf("\033[1;33m MPHASE_PICKRIGHTSIDE_LHHOLD \033[0m\n");
		//bEncCheck = true;
		this->Destime = this->Goalcnt / 200.0;

		// lh
		this->Despos[0] = 0.55;
		this->Despos[1] = 0.3;
		this->Despos[2] = 0.55;

		this->LHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

		this->Temprv1[0] = -70*D2R;
		this->Temprv1[1] = 1;
		this->Temprv1[2] = 0;
		this->Temprv1[3] = 0;
		this->RV2QT(this->Temprv1, this->Tempth1);

		this->Temprv2[0] = -80*D2R;
		this->Temprv2[1] = 0;
		this->Temprv2[2] = 1;
		this->Temprv2[3] = 0;
		this->RV2QT(this->Temprv2, this->Tempth2);

		QTCross(Tempth1, Tempth2, Desth);

		this->LHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

		Deselb = LElbCal(Despos, Desth, Deswst);
		this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

		break;






    case MPHASE_PICKRIGHTSIDE_WSTTURN_90:
        this->Goalcnt = 1000;
		printf("\033[1;33m MPHASE_PICKRIGHTSIDE_WSTTURN \033[0m\n");
		this->Destime = this->Goalcnt / 200.0;

        //WBIKframe(WBmotion, WBmode_RULU);

        WSTmoveset(WBmotion, -90.0, Destime, MODE_ABSOLUTE);

        PELZmoveset(WBmotion, 0.06, Destime, MODE_ABSOLUTE);

        printf("cur comx %.3f \n", WBmotion->pCOM_2x1[0]);
        //COMmoveset(WBmotion, 0.00, 0, Destime, MODE_ABSOLUTE);

		break;






	case MPHASE_PICKRIGHTSIDE_RHPUT:
        this->Goalcnt = 400;
		printf("\033[1;33m MPHASE_PICKRIGHTSIDE_RHPUT \033[0m\n");
		this->Destime = this->Goalcnt / 200.0;

        DockPoscal(DebS[0].Pos_r, DebS[0].Quat_r, Dock_Distance + Palm_Distance, Dockpos);
        RHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);
        RHOrimoveset(WBmotion, DebS[0].Quat_r, Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Dockpos, DebS[0].Quat_r, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

		break;


    case MPHASE_PICKRIGHTSIDE_RHNULL:
        this->Goalcnt = 10;
        printf("\033[1;33m MPHASE_PICKRIGHTSIDE_RHNULL \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        RBwFTsensorNull(2, 53);
        //RBwFTsensorNull(3, 54);

        break;


    case MPHASE_PICKRIGHTSIDE_RHDOWN:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE_PICKRIGHTSIDE_RHDOWN \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bRHFTon = true;

        DockPoscal(DebS[0].Pos_r, DebS[0].Quat_r, FT_Distance + Palm_Distance, Dockpos);
        RHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Dockpos, DebS[0].Quat_r, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);


        break;

    case MPHASE_PICKRIGHTSIDE_RHOFF:
        this->Goalcnt = 100;
        printf("\033[1;33m MPHASE_PICKRIGHTSIDE_RHOFF \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bRHFTon = false;

        bEncCheckRH = true;

        joint->SetMoveJoint(RHAND, -125, 10, MODE_ABSOLUTE);

        break;

    case MPHASE_PICKRIGHTSIDE_RHUP:
        this->Goalcnt = 300;
        printf("\033[1;33m MPHASE_PICKRIGHTSIDE_RHUP \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bEncCheckRH = false;

        DockPoscal(DebS[0].Pos_r, DebS[0].Quat_r, Dock_Distance + Palm_Distance, Dockpos);
        RHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);
        RHOrimoveset(WBmotion, DebS[0].Quat_r, Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Dockpos, DebS[0].Quat_r, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;

    case MPHASE_PICKRIGHTSIDE_RHON:
        this->Goalcnt = 100;
        printf("\033[1;33m MPHASE_PICKRIGHTSIDE_RHON \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        joint->SetMoveJoint(RHAND, 125, 10, MODE_ABSOLUTE);

        break;


    case MPHASE_PICKRIGHTSIDE_LHPUT:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE_PICKRIGHTSIDE_LHPUT \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        DockPoscal(DebS[1].Pos_r, DebS[1].Quat_r, Dock_Distance + Palm_Distance, Dockpos);
        LHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);
        LHOrimoveset(WBmotion, DebS[1].Quat_r, Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Dockpos, DebS[1].Quat_r, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;


    case MPHASE_PICKRIGHTSIDE_LHNULL:
        this->Goalcnt = 10;
        printf("\033[1;33m MPHASE_PICKRIGHTSIDE_LHNULL \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        //RBwFTsensorNull(2, 53);
        RBwFTsensorNull(3, 54);

        break;


    case MPHASE_PICKRIGHTSIDE_LHDOWN:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE_PICKRIGHTSIDE_LHDOWN \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bLHFTon = true;

        DockPoscal(DebS[1].Pos_r, DebS[1].Quat_r, FT_Distance + Palm_Distance, Dockpos);
        LHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Dockpos, DebS[1].Quat_r, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);


        break;

    case MPHASE_PICKRIGHTSIDE_LHOFF:
        this->Goalcnt = 500;
        printf("\033[1;33m MPHASE_PICKRIGHTSIDE_LHOFF \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bLHFTon = false;

        bEncCheckLH = true;

        joint->SetMoveJoint(LHAND, -125, 10, MODE_ABSOLUTE);

        break;

    case MPHASE_PICKRIGHTSIDE_LHUP:
        this->Goalcnt = 300;
        printf("\033[1;33m MPHASE_PICKRIGHTSIDE_LHUP \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bEncCheckLH = false;

        DockPoscal(DebS[1].Pos_r, DebS[1].Quat_r, Dock_Distance + Palm_Distance, Dockpos);
        LHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);
        LHOrimoveset(WBmotion, DebS[1].Quat_r, Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Dockpos, DebS[1].Quat_r, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;

    case MPHASE_PICKRIGHTSIDE_LHON:
        this->Goalcnt = 100;
        printf("\033[1;33m MPHASE_PICKRIGHTSIDE_LHON \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        joint->SetMoveJoint(LHAND, 125, 10, MODE_ABSOLUTE);

        break;

    case MPHASE_PICKRIGHTSIDE_HANDREADY:
        this->Goalcnt = 500;
        printf("\033[1;33m MPHASE_PICKRIGHTSIDE_HANDREADY \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        // rh
        this->Despos[0] = 0.55;
        this->Despos[1] = -0.3;
        this->Despos[2] = 0.45;

        this->RHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

        this->Desrv[0] = -45*D2R;
        this->Desrv[1] = 0;
        this->Desrv[2] = 1;
        this->Desrv[3] = 0;
        this->RV2QT(this->Desrv, this->Desth);

        this->RHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Despos, Desth, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        // lh
        this->Despos[0] = 0.55;
        this->Despos[1] = 0.3;
        this->Despos[2] = 0.45;

        this->LHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

        this->Desrv[0] = -45*D2R;
        this->Desrv[1] = 0;
        this->Desrv[2] = 1;
        this->Desrv[3] = 0;
        this->RV2QT(this->Desrv, this->Desth);

        this->LHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Despos, Desth, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        PELZmoveset(WBmotion, 0.12, Destime, MODE_ABSOLUTE);

        break;

    case MPHASE_PICKRIGHTSIDE_WSTTURN0:
        this->Goalcnt = 500;
        printf("\033[1;33m MPHASE_PICKRIGHTSIDE_WSTTURN0 \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        WSTmoveset(WBmotion, 0.0, Destime, MODE_ABSOLUTE);

        this->Despos[0] = WBmotion->pRH_3x1[0];
        this->Despos[1] = WBmotion->pRH_3x1[1];
        this->Despos[2] = WBmotion->pRH_3x1[2];

        this->Desrv[0] = -80*D2R;
        this->Desrv[1] = 0;
        this->Desrv[2] = 1;
        this->Desrv[3] = 0;
        this->RV2QT(this->Desrv, this->Desth);

        this->RHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Despos, Desth, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);


        this->Despos[0] = WBmotion->pLH_3x1[0];
        this->Despos[1] = WBmotion->pLH_3x1[1];
        this->Despos[2] = WBmotion->pLH_3x1[2];

        this->Desrv[0] = -80*D2R;
        this->Desrv[1] = 0;
        this->Desrv[2] = 1;
        this->Desrv[3] = 0;
        this->RV2QT(this->Desrv, this->Desth);

        this->LHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Despos, Desth, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;


    case MPHASE_PICKRIGHTSIDE_HOME:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE_PICKRIGHTSIDE_HOME \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        // rh
        this->Despos[0] = 0.291;
        this->Despos[1] = -0.241;
        this->Despos[2] = 0.221;

        this->RHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

        this->Desrv[0] = -90*D2R;
        this->Desrv[1] = 0;
        this->Desrv[2] = 1;
        this->Desrv[3] = 0;
        this->RV2QT(this->Desrv, this->Desth);

        this->RHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        this->RElbmoveset(WBmotion, -4.873, this->Destime, MODE_ABSOLUTE);

        // lh
        this->Despos[0] = 0.291;
        this->Despos[1] = 0.241;
        this->Despos[2] = 0.221;

        this->LHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

        this->Desrv[0] = -90*D2R;
        this->Desrv[1] = 0;
        this->Desrv[2] = 1;
        this->Desrv[3] = 0;
        this->RV2QT(this->Desrv, this->Desth);

        this->LHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        this->LElbmoveset(WBmotion, 4.873, this->Destime, MODE_ABSOLUTE);


        joint->SetMoveJoint(RHAND, 0, 10, MODE_ABSOLUTE);
        joint->SetMoveJoint(LHAND, 0, 10, MODE_ABSOLUTE);

        break;


        // ---------------------------------------------------------------------- //










    case MPHASE1_PICKRIGHTSIDE_0:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE1_PICKRIGHTSIDE_0 \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        // wst
//        Deswst = -50.0;
//        this->WSTmoveset(WBmotion, Deswst, this->Destime, MODE_ABSOLUTE);

        PELZmoveset(WBmotion, 0.12, Destime, MODE_ABSOLUTE);

        break;

    case MPHASE1_PICKRIGHTSIDE_READY:
        this->Goalcnt = 800;
        printf("\033[1;33m MPHASE1_PICKRIGHTSIDE_READY \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

            printf("RH pos : %.3f %.3f %.3f %.3f \n", WBmotion->pRH_3x1[0], WBmotion->pRH_3x1[1], WBmotion->pRH_3x1[2], WBmotion->RElb_ang*R2D);
            printf("LH pos : %.3f %.3f %.3f %.3f \n", WBmotion->pLH_3x1[0], WBmotion->pLH_3x1[1], WBmotion->pLH_3x1[2], WBmotion->LElb_ang*R2D);


        // wst
        Deswst = -0.0;
        this->WSTmoveset(WBmotion, Deswst, this->Destime, MODE_ABSOLUTE);

        // rh
        this->Despos[0] = 0.55;
        this->Despos[1] = -0.3;
        this->Despos[2] = 0.45;

        this->RHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

        this->Desrv[0] = -45*D2R;
        this->Desrv[1] = 0;
        this->Desrv[2] = 1;
        this->Desrv[3] = 0;
        this->RV2QT(this->Desrv, this->Desth);

        this->RHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Despos, Desth, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        // lh
        this->Despos[0] = 0.55;
        this->Despos[1] = 0.3;
        this->Despos[2] = 0.45;

        this->LHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

        this->Desrv[0] = -45*D2R;
        this->Desrv[1] = 0;
        this->Desrv[2] = 1;
        this->Desrv[3] = 0;
        this->RV2QT(this->Desrv, this->Desth);

        this->LHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Despos, Desth, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;

    case MPHASE1_PICKRIGHTSIDE_WSTTURN90:
        this->Goalcnt = 700;
        printf("\033[1;33m MPHASE1_PICKRIGHTSIDE_WSTTURN \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        WSTmoveset(WBmotion, 90.0, Destime, MODE_ABSOLUTE);
        PELZmoveset(WBmotion, -0.12, Destime, MODE_ABSOLUTE);

        break;

    case MPHASE1_PICKRIGHTSIDE_RH:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE1_PICKRIGHTSIDE_RH \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        DockPoscal(DebG[2].Pos_r, DebG[2].Quat_r, Dock_Distance + Palm_Distance, Dockpos);
        RHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);
        RHOrimoveset(WBmotion, DebG[2].Quat_r, Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Dockpos, DebG[2].Quat_r, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        joint->SetMoveJoint(RHAND, -125, 10, MODE_ABSOLUTE);

        break;

    case MPHASE1_PICKRIGHTSIDE_LH:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE1_PICKRIGHTSIDE_LH \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        DockPoscal(DebG[3].Pos_r, DebG[3].Quat_r, Dock_Distance + Palm_Distance, Dockpos);
        LHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);
        LHOrimoveset(WBmotion, DebG[3].Quat_r, Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Dockpos, DebG[3].Quat_r, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        joint->SetMoveJoint(LHAND, -125, 10, MODE_ABSOLUTE);

        break;



    case MPHASE1_PICKRIGHTSIDE_FINOPEN:
        this->Goalcnt = 100;
        printf("\033[1;33m MPHASE1_PICKRIGHTSIDE_FINOPEN \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bEncCheckRH = true;
        bEncCheckLH = true;

        RBwFTsensorNull(2, 53);
        RBwFTsensorNull(3, 54);

        joint->SetMoveJoint(RHAND, -125, 10, MODE_ABSOLUTE);
        joint->SetMoveJoint(LHAND, -125, 10, MODE_ABSOLUTE);


        break;

    case MPHASE1_PICKRIGHTSIDE_RHDOCK:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE1_PICKRIGHTSIDE_RHDOCK \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bEncCheckRH = false;
        bEncCheckLH = false;

        bRHFTon = true;
        bLHFTon = false;

        DockPoscal(DebG[2].Pos_r, DebG[2].Quat_r, FT_Distance + Palm_Distance, Dockpos);
        RHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Dockpos, DebG[2].Quat_r, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;

    case MPHASE1_PICKRIGHTSIDE_LHDOCK:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE1_PICKRIGHTSIDE_LHDOCK \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bRHFTon = false;
        bLHFTon = true;

        DockPoscal(DebG[3].Pos_r, DebG[3].Quat_r, FT_Distance + Palm_Distance, Dockpos);
        LHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Dockpos, DebG[3].Quat_r, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;

    case MPHASE1_PICKRIGHTSIDE_RHGRASP:
        this->Goalcnt = 10;
        printf("\033[1;33m MPHASE1_PICKRIGHTSIDE_RHGRASP \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bRHFTon = false;
        bLHFTon = false;

        joint->SetMoveJoint(RHAND, 125, 10, MODE_ABSOLUTE);

        break;

    case MPHASE1_PICKRIGHTSIDE_LHGRASP:
        this->Goalcnt = 10;
        printf("\033[1;33m MPHASE1_PICKRIGHTSIDE_LHGRASP \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        joint->SetMoveJoint(LHAND, 125, 10, MODE_ABSOLUTE);

        break;

    case MPHASE1_PICKRIGHTSIDE_FINCLOSE:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE1_PICKRIGHTSIDE_FINCLOSE \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        joint->SetMoveJoint(RHAND, 125, 10, MODE_ABSOLUTE);
        joint->SetMoveJoint(LHAND, 125, 10, MODE_ABSOLUTE);


        break;

    case MPHASE1_PICKRIGHTSIDE_RHLIFT:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE1_PICKRIGHTSIDE_RHLIFT \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        DockPoscal(DebG[2].Pos_r, DebG[2].Quat_r, Lift_Distance + Palm_Distance, Dockpos);
        RHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Dockpos, DebG[2].Quat_r, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;

    case MPHASE1_PICKRIGHTSIDE_RHHOLD:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE1_PICKRIGHTSIDE_RHHOLD \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        // rh
        this->Despos[0] = 0.55;
        this->Despos[1] = -0.3;
        this->Despos[2] = 0.55;

        this->RHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

        this->Temprv1[0] = 70*D2R;
        this->Temprv1[1] = 1;
        this->Temprv1[2] = 0;
        this->Temprv1[3] = 0;
        this->RV2QT(this->Temprv1, this->Tempth1);

        this->Temprv2[0] = -80*D2R;
        this->Temprv2[1] = 0;
        this->Temprv2[2] = 1;
        this->Temprv2[3] = 0;
        this->RV2QT(this->Temprv2, this->Tempth2);

        QTCross(Tempth1, Tempth2, Desth);

        this->RHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Despos, Desth, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;

    case MPHASE1_PICKRIGHTSIDE_LHLIFT:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE1_PICKRIGHTSIDE_LHLIFT \033[0m\n");
        //bEncCheck = true;
        this->Destime = this->Goalcnt / 200.0;

        DockPoscal(DebG[3].Pos_r, DebG[3].Quat_r, Lift_Distance + Palm_Distance, Dockpos);
        LHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Dockpos, DebG[3].Quat_r, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;

    case MPHASE1_PICKRIGHTSIDE_LHHOLD:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE1_PICKRIGHTSIDE_LHHOLD \033[0m\n");
        //bEncCheck = true;
        this->Destime = this->Goalcnt / 200.0;

        // lh
        this->Despos[0] = 0.55;
        this->Despos[1] = 0.3;
        this->Despos[2] = 0.55;

        this->LHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

        this->Temprv1[0] = -70*D2R;
        this->Temprv1[1] = 1;
        this->Temprv1[2] = 0;
        this->Temprv1[3] = 0;
        this->RV2QT(this->Temprv1, this->Tempth1);

        this->Temprv2[0] = -80*D2R;
        this->Temprv2[1] = 0;
        this->Temprv2[2] = 1;
        this->Temprv2[3] = 0;
        this->RV2QT(this->Temprv2, this->Tempth2);

        QTCross(Tempth1, Tempth2, Desth);

        this->LHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Despos, Desth, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        PELZmoveset(WBmotion, -0.00, Destime, MODE_ABSOLUTE);

        break;






    case MPHASE1_PICKRIGHTSIDE_WSTTURN_90:
        this->Goalcnt = 1000;
        printf("\033[1;33m MPHASE1_PICKRIGHTSIDE_WSTTURN \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        //WBIKframe(WBmotion, WBmode_RULU);

        WSTmoveset(WBmotion, -90.0, Destime, MODE_ABSOLUTE);

        printf("cur comx %.3f \n", WBmotion->pCOM_2x1[0]);
        //COMmoveset(WBmotion, 0.00, 0, Destime, MODE_ABSOLUTE);

        PELZmoveset(WBmotion, 0.06, Destime, MODE_ABSOLUTE);

        break;






    case MPHASE1_PICKRIGHTSIDE_RHPUT:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE1_PICKRIGHTSIDE_RHPUT \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        DockPoscal(DebS[2].Pos_r, DebS[2].Quat_r, Dock_Distance + Palm_Distance, Dockpos);
        RHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);
        RHOrimoveset(WBmotion, DebS[2].Quat_r, Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Dockpos, DebS[2].Quat_r, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;


    case MPHASE1_PICKRIGHTSIDE_RHNULL:
        this->Goalcnt = 10;
        printf("\033[1;33m MPHASE1_PICKRIGHTSIDE_RHNULL \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        RBwFTsensorNull(2, 53);
        //RBwFTsensorNull(3, 54);

        break;


    case MPHASE1_PICKRIGHTSIDE_RHDOWN:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE1_PICKRIGHTSIDE_RHDOWN \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bRHFTon = true;

        DockPoscal(DebS[2].Pos_r, DebS[2].Quat_r, FT_Distance + Palm_Distance, Dockpos);
        RHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Dockpos, DebS[2].Quat_r, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);


        break;

    case MPHASE1_PICKRIGHTSIDE_RHOFF:
        this->Goalcnt = 100;
        printf("\033[1;33m MPHASE1_PICKRIGHTSIDE_RHOFF \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bRHFTon = false;

        bEncCheckRH = true;

        joint->SetMoveJoint(RHAND, -125, 10, MODE_ABSOLUTE);

        break;

    case MPHASE1_PICKRIGHTSIDE_RHUP:
        this->Goalcnt = 300;
        printf("\033[1;33m MPHASE1_PICKRIGHTSIDE_RHUP \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bEncCheckRH = false;

        DockPoscal(DebS[2].Pos_r, DebS[2].Quat_r, Dock_Distance + Palm_Distance, Dockpos);
        RHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);
        RHOrimoveset(WBmotion, DebS[2].Quat_r, Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Dockpos, DebS[2].Quat_r, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;

    case MPHASE1_PICKRIGHTSIDE_RHON:
        this->Goalcnt = 100;
        printf("\033[1;33m MPHASE1_PICKRIGHTSIDE_RHON \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        joint->SetMoveJoint(RHAND, 125, 10, MODE_ABSOLUTE);

        break;


    case MPHASE1_PICKRIGHTSIDE_LHPUT:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE1_PICKRIGHTSIDE_LHPUT \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        DockPoscal(DebS[3].Pos_r, DebS[3].Quat_r, Dock_Distance + Palm_Distance, Dockpos);
        LHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);
        LHOrimoveset(WBmotion, DebS[3].Quat_r, Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Dockpos, DebS[3].Quat_r, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;


    case MPHASE1_PICKRIGHTSIDE_LHNULL:
        this->Goalcnt = 10;
        printf("\033[1;33m MPHASE1_PICKRIGHTSIDE_LHNULL \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        //RBwFTsensorNull(2, 53);
        RBwFTsensorNull(3, 54);

        break;


    case MPHASE1_PICKRIGHTSIDE_LHDOWN:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE1_PICKRIGHTSIDE_LHDOWN \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bLHFTon = true;

        DockPoscal(DebS[3].Pos_r, DebS[3].Quat_r, FT_Distance + Palm_Distance, Dockpos);
        LHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Dockpos, DebS[3].Quat_r, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);


        break;

    case MPHASE1_PICKRIGHTSIDE_LHOFF:
        this->Goalcnt = 500;
        printf("\033[1;33m MPHASE1_PICKRIGHTSIDE_LHOFF \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bLHFTon = false;

        bEncCheckLH = true;

        joint->SetMoveJoint(LHAND, -125, 10, MODE_ABSOLUTE);

        break;

    case MPHASE1_PICKRIGHTSIDE_LHUP:
        this->Goalcnt = 300;
        printf("\033[1;33m MPHASE1_PICKRIGHTSIDE_LHUP \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        bEncCheckLH = false;

        DockPoscal(DebS[3].Pos_r, DebS[3].Quat_r, Dock_Distance + Palm_Distance, Dockpos);
        LHPosmoveset(WBmotion, Dockpos, Destime, MODE_ABSOLUTE);
        LHOrimoveset(WBmotion, DebS[3].Quat_r, Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Dockpos, DebS[3].Quat_r, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;

    case MPHASE1_PICKRIGHTSIDE_LHON:
        this->Goalcnt = 100;
        printf("\033[1;33m MPHASE1_PICKRIGHTSIDE_LHON \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        joint->SetMoveJoint(LHAND, 125, 10, MODE_ABSOLUTE);

        break;

    case MPHASE1_PICKRIGHTSIDE_HANDREADY:
        this->Goalcnt = 500;
        printf("\033[1;33m MPHASE1_PICKRIGHTSIDE_HANDREADY \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        // rh
        this->Despos[0] = 0.55;
        this->Despos[1] = -0.3;
        this->Despos[2] = 0.45;

        this->RHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

        this->Desrv[0] = -45*D2R;
        this->Desrv[1] = 0;
        this->Desrv[2] = 1;
        this->Desrv[3] = 0;
        this->RV2QT(this->Desrv, this->Desth);

        this->RHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Despos, Desth, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        // lh
        this->Despos[0] = 0.55;
        this->Despos[1] = 0.3;
        this->Despos[2] = 0.45;

        this->LHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

        this->Desrv[0] = -45*D2R;
        this->Desrv[1] = 0;
        this->Desrv[2] = 1;
        this->Desrv[3] = 0;
        this->RV2QT(this->Desrv, this->Desth);

        this->LHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Despos, Desth, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;

    case MPHASE1_PICKRIGHTSIDE_WSTTURN0:
        this->Goalcnt = 500;
        printf("\033[1;33m MPHASE1_PICKRIGHTSIDE_WSTTURN0 \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        WSTmoveset(WBmotion, 0.0, Destime, MODE_ABSOLUTE);

        this->Despos[0] = WBmotion->pRH_3x1[0];
        this->Despos[1] = WBmotion->pRH_3x1[1];
        this->Despos[2] = WBmotion->pRH_3x1[2];

        this->Desrv[0] = -80*D2R;
        this->Desrv[1] = 0;
        this->Desrv[2] = 1;
        this->Desrv[3] = 0;
        this->RV2QT(this->Desrv, this->Desth);

        this->RHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        Deselb = RElbCal(Despos, Desth, Deswst);
        this->RElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);


        this->Despos[0] = WBmotion->pLH_3x1[0];
        this->Despos[1] = WBmotion->pLH_3x1[1];
        this->Despos[2] = WBmotion->pLH_3x1[2];

        this->Desrv[0] = -80*D2R;
        this->Desrv[1] = 0;
        this->Desrv[2] = 1;
        this->Desrv[3] = 0;
        this->RV2QT(this->Desrv, this->Desth);

        this->LHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        Deselb = LElbCal(Despos, Desth, Deswst);
        this->LElbmoveset(WBmotion, Deselb, this->Destime, MODE_ABSOLUTE);

        break;


    case MPHASE1_PICKRIGHTSIDE_HOME:
        this->Goalcnt = 400;
        printf("\033[1;33m MPHASE1_PICKRIGHTSIDE_HOME \033[0m\n");
        this->Destime = this->Goalcnt / 200.0;

        // rh
		this->Despos[0] = 0.391;
        this->Despos[1] = -0.241;
		this->Despos[2] = 0.121;

        this->RHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

        this->Desrv[0] = -90*D2R;
        this->Desrv[1] = 0;
        this->Desrv[2] = 1;
        this->Desrv[3] = 0;
        this->RV2QT(this->Desrv, this->Desth);

        this->RHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        this->RElbmoveset(WBmotion, -4.873, this->Destime, MODE_ABSOLUTE);

        // lh
		this->Despos[0] = 0.391;
        this->Despos[1] = 0.241;
		this->Despos[2] = 0.121;

        this->LHPosmoveset(WBmotion, this->Despos, this->Destime, MODE_ABSOLUTE);

        this->Desrv[0] = -90*D2R;
        this->Desrv[1] = 0;
        this->Desrv[2] = 1;
        this->Desrv[3] = 0;
        this->RV2QT(this->Desrv, this->Desth);

        this->LHOrimoveset(WBmotion, this->Desth, this->Destime, MODE_ABSOLUTE);

        this->LElbmoveset(WBmotion, 4.873, this->Destime, MODE_ABSOLUTE);


        joint->SetMoveJoint(RHAND, 0, 10, MODE_ABSOLUTE);
        joint->SetMoveJoint(LHAND, 0, 10, MODE_ABSOLUTE);

        break;

	}
}
