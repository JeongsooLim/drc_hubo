#include <iostream>
#include "BasicTrajectory.h"

using namespace std;
namespace rainbow{

//===================================================================================
//===================================================================================
// 1st Order Trajectory
//===================================================================================
//===================================================================================
int TRInfoOrder1::GetCurrentValue(TRInfo _info){
    currentAcceleration = ((TRInfoOrder1*)_info)->getCurrentAcceleration();
    currentVelocity = ((TRInfoOrder1*)_info)->getCurrentVelocity();
    currentPosition = ((TRInfoOrder1*)_info)->getCurrentPosition();
    return RB_SUCCESS;
}
int TRInfoOrder1::GetCurrentValue(doubles _retVal){	//this function will be called in special cases
                                                            //when there is no preceeding trajectory information
    currentPosition = _retVal[0];
    return RB_SUCCESS;
}


// Constant Trajectory Method------------------------------
int	TrajectoryConst::CalculateParameter(){
    return RB_SUCCESS;
}
doubles TrajectoryConst::CalculateTrajectory(double _nTime){
    retData[0] = currentPosition;
    currentVelocity = 0.0;
    currentAcceleration = 0.0;
    return retData;
}

// 3rd Polynomial Trajectory Method------------------------------
int	TrajectoryPoly3rd::CalculateParameter(){
    double polyTemp1, polyTemp2;
    polyTemp1 = goalPosition - currentVelocity - currentPosition;
    polyTemp2 = goalVelocity - currentVelocity;

    trajParam[0] = polyTemp2 - 2.0*polyTemp1;
    trajParam[1] = 3.0*polyTemp1 - polyTemp2;
    trajParam[2] = currentVelocity;
    trajParam[3] = currentPosition;
    return RB_SUCCESS;
}
doubles TrajectoryPoly3rd::CalculateTrajectory(double _nTime){
    currentPosition = trajParam[0]*_nTime*_nTime*_nTime
                        + trajParam[1]*_nTime*_nTime
                        + trajParam[2]*_nTime
                        + trajParam[3];
    currentVelocity = 3.0*trajParam[0]*_nTime*_nTime
                        + 2.0*trajParam[1]*_nTime
                        + trajParam[2];
    currentAcceleration = 6.0*trajParam[0]*_nTime + 2.0*trajParam[1];
    retData[0] = currentPosition;
    return retData;
}

// 5th Polynomial Trajectory Method------------------------------
int TrajectoryPoly5th::CalculateParameter(){
    double polyTemp1, polyTemp2, polyTemp3;
    polyTemp1 = goalPosition - 0.5*currentAcceleration - currentVelocity - currentPosition;
    polyTemp2 = goalVelocity - currentAcceleration - currentVelocity;
    polyTemp3 = goalAcceleration - currentAcceleration;

    trajParam[0] = 0.5*(polyTemp3 - 6.0*polyTemp2 + 12.0*polyTemp1);
    trajParam[1] = polyTemp2 - 3.0*polyTemp1 - 2.0*trajParam[0];
    trajParam[2] = polyTemp1 - trajParam[0] - trajParam[1];
    trajParam[3] = 0.5*currentAcceleration;
    trajParam[4] = currentVelocity;
    trajParam[5] = currentPosition;
    return RB_SUCCESS;
}
doubles TrajectoryPoly5th::CalculateTrajectory(double _nTime){
    currentPosition = trajParam[0]*_nTime*_nTime*_nTime*_nTime*_nTime
                        + trajParam[1]*_nTime*_nTime*_nTime*_nTime
                        + trajParam[2]*_nTime*_nTime*_nTime
                        + trajParam[3]*_nTime*_nTime
                        + trajParam[4]*_nTime
                        + trajParam[5];
    currentVelocity = 5.0*trajParam[0]*_nTime*_nTime*_nTime*_nTime
                        + 4.0*trajParam[1]*_nTime*_nTime*_nTime
                        + 3.0*trajParam[2]*_nTime*_nTime
                        + 2.0*trajParam[3]*_nTime
                        + trajParam[4];
    currentAcceleration = 20.0*trajParam[0]*_nTime*_nTime*_nTime
                        + 12.0*trajParam[1]*_nTime*_nTime
                        + 6.0*trajParam[2]*_nTime
                        + 2.0*trajParam[3];

    retData[0] = currentPosition;
    return retData;
}

// Cosine Trajectory Method---------------------------------------
int TrajectoryCosine::CalculateParameter(){
    trajParam[0] = currentPosition;
    trajParam[1] = goalPosition - currentPosition;
    return RB_SUCCESS;
}
doubles TrajectoryCosine::CalculateTrajectory(double _nTime){
    currentPosition = trajParam[0] + trajParam[1]*0.5*(1.0-cos(PIf*_nTime));
    currentVelocity = 0.5*trajParam[1]*PIf*sin(PIf*_nTime);
    currentAcceleration = 0.5*trajParam[1]*PIf*PIf*cos(PIf*_nTime);
    retData[0] = currentPosition;
    return retData;
}



//===================================================================================
//===================================================================================
// Quat Order Trajectory
//===================================================================================
//===================================================================================
int TRInfoOrderQuat::GetCurrentValue(TRInfo _info){
    currentQuat = ((TRInfoOrderQuat*)_info)->getCurrentQuat();
    return RB_SUCCESS;
}
int TRInfoOrderQuat::GetCurrentValue(doubles _retVal){	//this function will be called in special cases
                                                        //when there is no preceeding trajectory information
    currentQuat = _retVal;
    return RB_SUCCESS;
}

// Slerp with Exponential---------------------------------------
int TrajectorySlerpExp::CalculateParameter(){
	quat q0,q1,qtemp;
	startQuat = currentQuat;
	q0.w=currentQuat[0];	q0.x=currentQuat[1];	q0.y=currentQuat[2];	q0.z=currentQuat[3];
	q1.w=goalQuat[0];		q1.x=goalQuat[1];		q1.y=goalQuat[2];		q1.z=goalQuat[3];
	q0 = unit(q0);
	q1 = unit(q1);
	inverse(q0);
	qtemp = q0*q1;

	trajParam[0] = acos(qtemp.w);
	trajParam[1] = qtemp.x/sin(trajParam[0]);
	trajParam[2] = qtemp.y/sin(trajParam[0]);
	trajParam[3] = qtemp.z/sin(trajParam[0]);
	return RB_SUCCESS;
}
doubles TrajectorySlerpExp::CalculateTrajectory(double _nTime){
    quat q0,q1,qtemp;
    q0.w=startQuat[0];	q0.x=startQuat[1];	q0.y=startQuat[2];	q0.z=startQuat[3];
    q1.w = cos(_nTime*trajParam[0]);
    q1.x = trajParam[1]*sin(_nTime*trajParam[0]);
    q1.y = trajParam[2]*sin(_nTime*trajParam[0]);
    q1.z = trajParam[3]*sin(_nTime*trajParam[0]);
    qtemp = q0*q1;
    retData[0] = qtemp.w;
    retData[1] = qtemp.x;
    retData[2] = qtemp.y;
    retData[3] = qtemp.z;
	currentQuat = retData;
    return retData;
}

// Slerp without Exponential-------------------------------------
int TrajectorySlerpNoExp::CalculateParameter(){
	quat q0,q1;
	double ftemp;
	startQuat = currentQuat;
	q0.w=currentQuat[0];	q0.x=currentQuat[1];	q0.y=currentQuat[2];	q0.z=currentQuat[3];
	q1.w=goalQuat[0];		q1.x=goalQuat[1];		q1.y=goalQuat[2];		q1.z=goalQuat[3];
	q0 = unit(q0);
	q1 = unit(q1);
	ftemp = dot(q0,q1);

    if(fabs(ftemp) > 1.0)
        trajParam = 0.0;
    else
        trajParam = acos(ftemp);
	return RB_SUCCESS;
}
doubles TrajectorySlerpNoExp::CalculateTrajectory(double _nTime){
	quat q0,q1,qtemp;
	double ftemp1,ftemp2;
	q0.w=startQuat[0];	q0.x=startQuat[1];	q0.y=startQuat[2];	q0.z=startQuat[3];
	q1.w=goalQuat[0];	q1.x=goalQuat[1];	q1.y=goalQuat[2];	q1.z=goalQuat[3];

	ftemp1 = sin((1.0-_nTime)*trajParam);
	ftemp2 = sin(_nTime*trajParam);


	if(sin(trajParam) < 0.001)
		retData = currentQuat;
	else{
		qtemp.w = (q0.w*ftemp1 + q1.w*ftemp2)/sin(trajParam);
		qtemp.x = (q0.x*ftemp1 + q1.x*ftemp2)/sin(trajParam);
		qtemp.y = (q0.y*ftemp1 + q1.y*ftemp2)/sin(trajParam);
		qtemp.z = (q0.z*ftemp1 + q1.z*ftemp2)/sin(trajParam);

		retData[0] = qtemp.w;
		retData[1] = qtemp.x;
		retData[2] = qtemp.y;
		retData[3] = qtemp.z;
	}
	currentQuat = retData;
	return retData;
}

// Euler Interpolation Trajectory Info----------------------------------
int TrajectoryQuatEuler::CalculateParameter(){
    startQuat = currentQuat;
    return RB_SUCCESS;
}
doubles TrajectoryQuatEuler::CalculateTrajectory(double _nTime){
    quat start_quat, goal_quat;
    for(int k=0;k<4;k++)
    {
        start_quat[k]=startQuat[k];
        goal_quat[k]=goalQuat[k];
    }
    double start_quat_double[4], start_quat_inv_double[4];
    for(int k=0;k<4;k++)
    {
        start_quat_double[k]=start_quat[k];
    }
    iQTinv(start_quat_double, start_quat_inv_double);
    quat start_quat_inv;
    for(int k=0;k<4;k++)
    {
        start_quat_inv[k]=start_quat_inv_double[k];
    }
    quat delta_quat=start_quat_inv*goal_quat;

    double rv[4];
    double delta_quat_d[4];
    for(int k=0;k<4;k++)
        delta_quat_d[k]=delta_quat[k];
    iQT2RV(delta_quat_d, rv);

    rv[0] = rv[0]*0.5f*(1.0f-cosf(PIf*_nTime));

    iRV2QT(rv, delta_quat_d);

    for(int k=0;k<4;k++)
        delta_quat[k]=delta_quat_d[k];

    quat result=start_quat*delta_quat;
    for(int k=0;k<4;k++)
        retData[k]=result[k];

    currentQuat = retData;
    return retData;
}
int iQTinv(const double *qt_4x1, double *result_4x1)
{
    result_4x1[0] = qt_4x1[0]/((qt_4x1[0])*(qt_4x1[0])+(qt_4x1[1])*(qt_4x1[1])+(qt_4x1[2])*(qt_4x1[2])+(qt_4x1[3])*(qt_4x1[3]));
    result_4x1[1] = -qt_4x1[1]/((qt_4x1[0])*(qt_4x1[0])+(qt_4x1[1])*(qt_4x1[1])+(qt_4x1[2])*(qt_4x1[2])+(qt_4x1[3])*(qt_4x1[3]));
    result_4x1[2] = -qt_4x1[2]/((qt_4x1[0])*(qt_4x1[0])+(qt_4x1[1])*(qt_4x1[1])+(qt_4x1[2])*(qt_4x1[2])+(qt_4x1[3])*(qt_4x1[3]));
    result_4x1[3] = -qt_4x1[3]/((qt_4x1[0])*(qt_4x1[0])+(qt_4x1[1])*(qt_4x1[1])+(qt_4x1[2])*(qt_4x1[2])+(qt_4x1[3])*(qt_4x1[3]));

    return 0;
}
int iQT2RV(const double *qt_4x1, double *rv)
{
    double EPS (2.e-6);

    double temp;
    rv[0] = acosf(qt_4x1[0])*2.f;

    if(fabs(sinf(rv[0]/2.f)) < EPS)
    {
        rv[1] = qt_4x1[1];
        rv[2] = qt_4x1[2];
        rv[3] = qt_4x1[3];
    }
    else
    {
        rv[1] = qt_4x1[1]/sinf(rv[0]/2.f);
        rv[2] = qt_4x1[2]/sinf(rv[0]/2.f);
        rv[3] = qt_4x1[3]/sinf(rv[0]/2.f);


        temp = sqrt(rv[1]*rv[1]+rv[2]*rv[2]+rv[3]*rv[3]);
        rv[1] /= temp;
        rv[2] /= temp;
        rv[3] /= temp;
    }

    return 0;
}
int iRV2QT(const double *rv, double *qt_4x1)
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

    return 0;
}
// Constant Quaternion Trajectory Info----------------------------------
int TrajectoryQuatConst::CalculateParameter(){
	return RB_SUCCESS;
}
doubles TrajectoryQuatConst::CalculateTrajectory(double _nTime){
	return currentQuat;
}

//===================================================================================
//===================================================================================
// Trajectory Controlling
//===================================================================================
//===================================================================================

// Constructor------------------------------------
TrajectoryHandler::TrajectoryHandler(int _order, double _tick){
    trajectoryOrder = _order;
    tickTime = _tick;
    trInfos.clear();
    currentInfo = NULL;
    AllocateRetValue();
}

void TrajectoryHandler::AllocateRetValue(){
    switch(trajectoryOrder){
    case ORDER_1:
        AllocateData(retValue, 1);
        break;
    case ORDER_2:
        AllocateData(retValue, 2);
        break;
    case ORDER_QUAT:
        AllocateData(retValue, 4);
        break;
    }
}

// Control Method for Trajectory Information-------------------
int TrajectoryHandler::AddTrajInfo(TRInfo _info){
    if(_info->trajectoryOrder != trajectoryOrder)
        return RB_FAIL;
    trInfos.push_back(_info);
    return RB_SUCCESS;
}
int TrajectoryHandler::InsertTrajInfo(TRInfo _info, int n){
    if(_info->trajectoryOrder != trajectoryOrder)
        return RB_FAIL;
    trInfos.insert(n, _info);
    return RB_SUCCESS;
}
int TrajectoryHandler::DeleteTrajInfo(int n){
    TRInfo tempInfo = trInfos.at(n);
    trInfos.remove(n);
    delete tempInfo;
    return RB_SUCCESS;
}
int TrajectoryHandler::OverwriteTrajInfo(TRInfo _info){
    if(_info->trajectoryOrder != trajectoryOrder)
        return RB_FAIL;
    if(currentInfo == NULL){
        currentInfo = _info;
        currentInfo->GetCurrentValue(retValue);
        currentInfo->CalculateParameter();
        TimeReset();
    }else{
        TRInfo tempInfo = currentInfo;
        currentInfo = _info;
        currentInfo->GetCurrentValue(tempInfo);
        currentInfo->CalculateParameter();
        TimeReset();
        delete tempInfo;
    }
    return RB_SUCCESS;
}

// Update the Trajectory-----------------------------------------
doubles TrajectoryHandler::UpdateTrajectory(){
    if(currentInfo == NULL){
        if(!trInfos.empty()){
            currentInfo = trInfos.first();
            currentInfo->GetCurrentValue(retValue);
            trInfos.pop_front();
            currentInfo->CalculateParameter();
            TimeReset();
            return UpdateTrajectory();
        }
    }else if(normalizedTime < 1.0){
        currentTime += tickTime;
        normalizedTime = currentTime/currentInfo->goalTime;
        if(normalizedTime > 1.0){
            normalizedTime = 1.0;
            retValue = currentInfo->CalculateTrajectory(normalizedTime);
            return UpdateTrajectory();
        }
        retValue = currentInfo->CalculateTrajectory(normalizedTime);
    }else{
        retValue = currentInfo->retData;
        if(!trInfos.empty()){
            TRInfo tempInfo = currentInfo;
            currentInfo = trInfos.first();
            currentInfo->GetCurrentValue(tempInfo);
            trInfos.pop_front();
            currentInfo->CalculateParameter();
            TimeReset();
            delete tempInfo;
            return UpdateTrajectory();
        }else currentInfo = NULL;
    }
    return retValue;
}

void TrajectoryHandler::StopAndEraseAll(){
    trInfos.clear();
    currentInfo = NULL;
}

}//end namespace rainbow


