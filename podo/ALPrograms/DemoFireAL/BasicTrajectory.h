#ifndef BASICTRAJECTORY_H
#define BASICTRAJECTORY_H

#include "BasicMatrix.h"

namespace rainbow{
const int RB_SUCCESS = true;
const int RB_FAIL = false;

const int ORDER_1		= 1;		// first order trajectory
const int ORDER_2		= 2;		// second order trajectory
const int ORDER_3		= 3;		// third order trajectory
const int ORDER_QUAT	= 4;		// quaternion trajectory


class TrajectoryInfo;
class TrajectoryHandler;

typedef TrajectoryInfo*				TRInfo;
typedef TrajectoryHandler*			TRHandler;

typedef QVector<TrajectoryInfo*>	TRInfos;
typedef QVector<double>				doubles;
typedef QVector<doubles>            doubless;
typedef QVector<int>				ints;
typedef QVector<ints>               intss;



inline void AllocateData(doubles &vec, const int n){
    vec.clear();
    for(int i=0; i<n; i++) vec.push_back(0.0);
}


inline doubles MakeQuat(double rollDeg, double pitchDeg, double yawDeg){
	quat q = quat::rotateX(rollDeg*D2Rf) * quat::rotateY(pitchDeg*D2Rf) * quat::rotateZ(yawDeg*D2Rf);
	doubles ret(4);
	ret[0] = q.w;
	ret[1] = q.x;
	ret[2] = q.y;
	ret[3] = q.z;
	return ret;
}
inline doubles MakeQuat(doubles preQuat, double rollDeg, double pitchDeg, double yawDeg){
	quat preQ(preQuat[0], preQuat[1], preQuat[2], preQuat[3]);
	quat q = preQ * quat::rotateX(rollDeg*D2Rf) * quat::rotateY(pitchDeg*D2Rf) * quat::rotateZ(yawDeg*D2Rf);
	doubles ret(4);
	ret[0] = q.w;
	ret[1] = q.x;
	ret[2] = q.y;
	ret[3] = q.z;
	return ret;
}
inline doubles MakeQuat(double qwDeg, double qx, double qy, double qz){
	quat q(qwDeg*D2Rf, qx, qy,qz);
	doubles ret(4);
	ret[0] = q.w;
	ret[1] = q.x;
	ret[2] = q.y;
	ret[3] = q.z;
	return ret;
}

// Base Abstract Class for Various Trajecoties============================================================
class TrajectoryInfo{
    friend class TrajectoryHandler;
protected:
    int		trajectoryOrder;	// the order should be same within the sequence of trajectories
    double	goalTime;
    doubles	retData;

    void	AllocateRetData(int n)				{AllocateData(retData, n);}
public:
    virtual int		CalculateParameter() = 0;					//pure virtual
    virtual doubles	CalculateTrajectory(double _nTime) = 0;		//pure virtual
    virtual int		GetCurrentValue(TRInfo _info) = 0;			//pure virtual
    virtual int		GetCurrentValue(doubles _retVal) = 0;		//pure virtual
};
//========================================================================================================


//========================================================================================================
// 1st Order Trajectory Information---------------------------------------------------
// now it demands current position, velocity, acceleration for the next trajectory
class TRInfoOrder1 : public TrajectoryInfo
{
protected:
    double currentPosition;
    double currentVelocity;
    double currentAcceleration;
public:
    TRInfoOrder1()								{trajectoryOrder = ORDER_1;	AllocateRetData(1); currentPosition = 0; currentVelocity = 0; currentAcceleration = 0;}
    double			getCurrentPosition()		{return currentPosition;}
    double			getCurrentVelocity()		{return currentVelocity;}
    double			getCurrentAcceleration()	{return currentAcceleration;}

    virtual int		GetCurrentValue(TRInfo _info);
    virtual int		GetCurrentValue(doubles _retVal);
};
// Quat Order Trajectory Information---------------------------------------------------
// now it demands current quaternion for the next trajectory
class TRInfoOrderQuat : public TrajectoryInfo
{
protected:
    doubles		currentQuat;
public:
    TRInfoOrderQuat()							{trajectoryOrder = ORDER_QUAT;	AllocateRetData(4);}
    doubles			getCurrentQuat()			{return currentQuat;}

    virtual int		GetCurrentValue(TRInfo _info);
    virtual int		GetCurrentValue(doubles _retVal);
};


//========================================================================================================
// Constant Trajectory Info------------------------------
class TrajectoryConst : public TRInfoOrder1{
private:

public:
    TrajectoryConst(double _time)				{goalTime = _time; AllocateRetData(1);}

    virtual int		CalculateParameter();
    virtual doubles	CalculateTrajectory(double _nTime);
};
// 3rd Polynomial Trajectory Info------------------------------
class TrajectoryPoly3rd : public TRInfoOrder1{
private:
    double	goalPosition;
    double	goalVelocity;

    double	trajParam[4];
public:
    TrajectoryPoly3rd(double _time, double _pos, double _vel)
                                                {goalTime = _time;	goalPosition = _pos;	goalVelocity = _vel;}

    virtual int		CalculateParameter();
    virtual doubles	CalculateTrajectory(double _nTime);
};
// 5th Polynomial Trajectory Info------------------------------
class TrajectoryPoly5th : public TRInfoOrder1{
private:
    double	goalPosition;
    double	goalVelocity;
    double	goalAcceleration;

    double	trajParam[6];
public:
    TrajectoryPoly5th(double _time, double _pos, double _vel, double _acc)
                                                {goalTime = _time;	goalPosition = _pos;	goalVelocity = _vel;	goalAcceleration = _acc;}

    virtual int		CalculateParameter();
    virtual doubles	CalculateTrajectory(double _nTime);
};
// Cosine Trajectory Info---------------------------------------
class TrajectoryCosine : public TRInfoOrder1{
private:
    double	goalPosition;

    double	trajParam[2];
public:
    TrajectoryCosine(double _time, double _pos)	{goalTime = _time;	goalPosition = _pos;}

    virtual int		CalculateParameter();
    virtual doubles	CalculateTrajectory(double _nTime);
};
//========================================================================================================


//========================================================================================================
// Slerp with Exponential Trajectory Info------------------------------
class TrajectorySlerpExp : public TRInfoOrderQuat{
private:
    doubles	goalQuat;
    doubles	startQuat;

    double	trajParam[4];
public:
    TrajectorySlerpExp(double _time, doubles _quat)	{goalTime = _time;	goalQuat = _quat;	AllocateData(startQuat,4);}

    virtual int		CalculateParameter();
    virtual doubles	CalculateTrajectory(double _nTime);
};
// Slerp without Exponential Trajectory Info----------------------------
class TrajectorySlerpNoExp : public TRInfoOrderQuat{
private:
    doubles	goalQuat;
    doubles	startQuat;

    double	trajParam;
public:
    TrajectorySlerpNoExp(double _time, doubles _quat)	{goalTime = _time;		goalQuat = _quat;}

    virtual int		CalculateParameter();
    virtual doubles	CalculateTrajectory(double _nTime);
};
// Euler Interpolation Trajectory Info---------------------------------
class TrajectoryQuatEuler : public TRInfoOrderQuat{
private:
    doubles goalQuat;
    doubles startQuat;

    double  trajParam;
public:
    TrajectoryQuatEuler(double _time, doubles _quat)    {goalTime = _time;      goalQuat = _quat;}

    virtual int     CalculateParameter();
    virtual doubles CalculateTrajectory(double _nTime);
};
// Constant Quaternion Trajectory Info----------------------------------
class TrajectoryQuatConst : public TRInfoOrderQuat{
private:
	doubles goalQuat;
	doubles startQuat;

public:
	TrajectoryQuatConst(double _time)	{goalTime = _time;}

	virtual int		CalculateParameter();
	virtual doubles	CalculateTrajectory(double _nTime);
};
//========================================================================================================



//========================================================================================================
// Handling Class for controlling TRInfos..
class TrajectoryHandler
{
private:
    int			trajectoryOrder;
    double		tickTime;

    double		currentTime;
    double		normalizedTime;

    TRInfos		trInfos;
    TRInfo		currentInfo;
    doubles		retValue;

    void	AllocateRetValue();
    void	TimeReset()				{currentTime = normalizedTime = 0.0;}

public:
    explicit TrajectoryHandler(int _order = ORDER_1, double _tick = 0.005);

    // Control Method for Trajectory Information-------------
    int		AddTrajInfo(TRInfo _info);
    int		InsertTrajInfo(TRInfo _info, int n);
    int		DeleteTrajInfo(int n);
    int		OverwriteTrajInfo(TRInfo _info);

    // Update the Trajectory---------------------------------
    doubles	UpdateTrajectory();
    void    StopAndEraseAll();

    int		SetRetValue(doubles _ret)	{if(currentInfo != NULL) return RB_FAIL;
                                        retValue = _ret; return RB_SUCCESS;}
	int		SetRetValue(double _ret)	{if(currentInfo != NULL) return RB_FAIL;
										doubles temp(1); temp[0] = _ret;
										retValue = temp; return RB_SUCCESS;}
    doubles	GetRetValue()				{return retValue;}
};
//========================================================================================================

int iQTinv(const double *qt_4x1, double *result_4x1);
int iQT2RV(const double *qt_4x1, double *rv);
int iRV2QT(const double *rv, double *qt_4x1);
}//end namespace rainbow

#endif // BASICTRAJECTORY_H
