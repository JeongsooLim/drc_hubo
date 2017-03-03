#ifndef ISNL_MATH_INTERPOLATION_H
#define ISNL_MATH_INTERPOLATION_H
#include <vector>
#include <assert.h>
#include <isnl/math/geometry.h>

namespace isnl{

// cubic spline based interpolation function
// to generate smooth spline curve which has "0 slope" on the start/end position
// t : current time
// time : time stemp for each via points : should be accending order, 0 for time[0]
// path : sequence of via points
// temp : temporally variable stores intermediate value (accelleration value)
void  prepInterpSSpline(const floats& time, const floats& path, floats& temp);
float interpSSpline(float t, const floats& time, const floats& path, const floats& temp);

void  prepInterpSSpline(const floats& time, const pos_s& path, pos_s& temp);
pos   interpSSpline(float t, const floats& time, const pos_s& path, const pos_s& temp);

float interpOneCos(float init, float goal, float t, float tmax);
pos   interpOneCos(const pos& init, const pos& goal, float t, float tmax);

// multiple points one-cosine interpolation
float interpOneCos(float t, const floats& time, const floats& path);
pos   interpOneCos(float t, const floats& time, const pos_s& path);

// linear interpolation
float interpLinear(float t, const floats& time, const floats& path);
pos   interpLinear(float t, const floats& time, const pos_s& path);



inline float interpOneCos(float init, float goal, float t, float tmax){
	if(t < 0.f){
		return init;
	}else if(t >= tmax){
		return goal;
	}

	return init + 0.5f*(1.f - cosf(t*PI/tmax))*(goal-init);
}
inline pos   interpOneCos(const pos& init, const pos& goal, float t, float tmax){
	if(t < 0.f){
		return init;
	}else if(t >= tmax){
		return goal;
	}
	float a = 0.5f*(1.f - cosf(t*PI/tmax));

	pos ret;
	ret.p = init.p + a*(goal.p-init.p);
	quat del = goal.q/init.q;
	if(norm(del.r) < 0.001){
		ret.q = init.q;
	}else{
		float ang = a*acosf(del.w)*2.f;
		ret.q = quat::rotate(ang, del.r)*init.q;
	}
	return ret;
}


inline void  prepInterpSSpline(const floats& x, const floats& y, floats& a){
	assert(x.size() == y.size());
	assert(x.size() > 1);
	int n = x.size();
	a.resize(n);

	floats  u(n+1);
	float dx, dy;

	// assume initial & final velocity(slope) is 0;
	float vy0 = 0, vyn = 0;

	dx =  x[1] - x[0];
	dy =  y[1] - y[0];
	a[0] = (vy0 > 1E30f ? 0.0f : -0.5f);
	u[0] = (vy0 > 1E30f ? 0.0f : ( (3.0f/dx) * (dy/dx -vy0) ));

	dx =  x[n-1] - x[n-2];
	dy =  y[n-1] - y[n-2];
	float qn = (vyn > 1E30f ? 0.0f : 0.5f);
	float un = (vyn > 1E30f ? 0.0f : (3.0f/dx)*(vyn - dy/dx)  );

	for(int i = 1; i < n-1; ++i){
		float sig = (x[i] - x[i-1]) / (x[i+1] - x[i-1]);
		float p   = sig * a[i-1] + 2.0f;
		a[i] = (sig - 1.0f)/p;
		u[i] = (y[i+1] - y[i])/(x[i+1] - x[i]) - (y[i] - y[i-1])/(x[i] - x[i-1]);
		u[i] = (6.0f*u[i]/(x[i+1] - x[i-1]) - sig*u[i-1])/p;
	}

	a[n-1] = (un-qn*u[n-2])/(qn*a[n-2] + 1.0f);

	for(int i = n-2; i>=0; --i){
		a[i] = a[i] * a[i+1] + u[i];
	}
}
inline float interpSSpline(float t, const floats& x, const floats& y, const floats& a){
	int n = x.size();

	int k;
	int i = 0;
	int j = n-1;

	// binary search
	while (j - i > 1) {k = (j+i) >> 1;(x[k] > t ? j=k : i=k);}

	float d = x[j] - x[i];
	float c1 = (x[j] - t)/d;
	float c2 = (t - x[i])/d;
	float c3 = (d*d/6.0f);
	float c4 = c3*(c1*c1*c1-c1);
	float c5 = c3*(c2*c2*c2-c2);

	return c1*y[i] + c2*y[j] + c4*a[i] + c5*a[j];
}
inline void  prepInterpSSpline(const floats& x, const pos_s& y, pos_s& a){
	assert(x.size() == y.size());
	assert(x.size() > 1);
	int n = x.size();
	a.resize(n);

	floats  u(n+1);
	float dx, dy;

	// assume initial & final velocity(slope) is 0;
	float vy0 = 0, vyn = 0;

	for(int l = 0; l < 3; ++l){
		dx =  x[1]      - x[0];
		dy =  y[1].v[l] - y[0].v[l];
		a[0].v[l] = (vy0 > 1E30f ? 0.0f : -0.5f);
		u[0]      = (vy0 > 1E30f ? 0.0f : ( (3.0f/dx) * (dy/dx -vy0) ));

		dx =  x[n-1]      - x[n-2];
		dy =  y[n-1].v[l] - y[n-2].v[l];
		float qn = (vyn > 1E30f ? 0.0f : 0.5f);
		float un = (vyn > 1E30f ? 0.0f : (3.0f/dx)*(vyn - dy/dx)  );

		for(int i = 1; i < n-1; ++i){
			float sig = (x[i] - x[i-1]) / (x[i+1] - x[i-1]);
			float p   = sig * a[i-1].v[l] + 2.0f;
			a[i].v[l] = (sig - 1.0f)/p;
			u[i]      = (y[i+1].v[l] - y[i].v[l])/(x[i+1] - x[i]) - (y[i].v[l] - y[i-1].v[l])/(x[i] - x[i-1]);
			u[i]      = (6.0f*u[i]/(x[i+1] - x[i-1]) - sig*u[i-1])/p;
		}

		a[n-1].v[l] = (un-qn*u[n-2])/(qn*a[n-2].v[l] + 1.0f);

		for(int i = n-2; i>=0; --i){
			a[i].v[l] = a[i].v[l] * a[i+1].v[l] + u[i];
		}
	}
}
inline pos   interpSSpline(float t, const floats& x, const pos_s& y, const pos_s& a){
	int n = x.size();

	int k;
	int i = 0;
	int j = n-1;

	// binary search
	while (j - i > 1) {k = (j+i) >> 1;(x[k] > t ? j=k : i=k);}

	float d = x[j] - x[i];
	float c1 = (x[j] - t)/d;
	float c2 = (t - x[i])/d;
	float c3 = (d*d/6.0f);
	float c4 = c3*(c1*c1*c1-c1);
	float c5 = c3*(c2*c2*c2-c2);
	
	pos ret(
			c1*y[i].x + c2*y[j].x + c4*a[i].x + c5*a[j].x,
			c1*y[i].y + c2*y[j].y + c4*a[i].y + c5*a[j].y,
			c1*y[i].z + c2*y[j].z + c4*a[i].z + c5*a[j].z
		);

	// interpolate quaternion linearly from y[0] to y[n-1]; ignore y[1] to y[n-2]
	const quat_& init = y[0].q;
	const quat_& goal = y[n-1].q;
	float tmax = x[n-1];
	float e = 0.5f*(1.0f - cosf(t*M_PI/tmax));

	quat del = goal/init;
	if(norm(del.r) < 0.001f){
		ret.q = init;
	}else{
		float ang = e*acosf(del.w)*2.f;
		ret.q = isnl::quat::rotate(ang, del.r)*init;
	}

	return ret;
}

// multiple points one-cosine interpolation
inline float interpOneCos(float t, const floats& x, const floats& y){
	int n = x.size();

	int k;
	int i = 0;
	int j = n-1;

	// binary search
	while (j - i > 1) {k = (j+i) >> 1;(x[k] > t ? j=k : i=k);}

	return interpOneCos(y[i], y[j], t - x[i], x[j] - x[i]);
}
inline pos   interpOneCos(float t, const floats& x, const pos_s& y){
	int n = x.size();

	int k;
	int i = 0;
	int j = n-1;

	// binary search
	while (j - i > 1) {k = (j+i) >> 1;(x[k] > t ? j=k : i=k);}

	return interpOneCos(y[i], y[j], t - x[i], x[j] - x[i]);
}

// linear interpolation
inline float interpLinear(float t, const floats& x, const floats& y){
	int n = x.size();

	int k;
	int i = 0;
	int j = n-1;

	// binary search
	while (j - i > 1) {k = (j+i) >> 1;(x[k] > t ? j=k : i=k);}

	return ((t-x[i])*y[j] + (x[j]-t)*y[i])/(x[j]-x[i]);
}
inline pos   interpLinear(float t, const floats& x, const pos_s& y){
	int n = x.size();

	int k;
	int i = 0, j = n-1;

	// binary search
	while (j - i > 1) {k = (j+i) >> 1;(x[k] > t ? j=k : i=k);}

	pos init = y[i], goal = y[j];
	float a = (t-x[i])/(x[j]-x[i]);

	pos ret;
	ret.p = init.p + a*(goal.p-init.p);
	quat del = goal.q/init.q;
	if(norm(del.r) < 0.001){
		ret.q = init.q;
	}else{
		float ang = a*acosf(del.w)*2.f;
		ret.q = quat::rotate(ang, del.r)*init.q;
	}
	return ret;
}

}
#endif
