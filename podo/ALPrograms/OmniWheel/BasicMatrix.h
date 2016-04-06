#ifndef BASICMATRIX_H
#define BASICMATRIX_H

#include <QVector>
#include "BasicMath.h"

class vec3;
class vec4;
class mat3;
class mat4;

class quat;
class pos;
class rpy;	// euler angle: roll, pitch, yaw
class pse;	// position & euler angle

typedef QVector<vec3>	vec3s;
typedef QVector<vec4>	vec4s;
typedef QVector<mat3>	mat3s;
typedef QVector<mat4>	mat4s;
typedef QVector<quat>	quats;
typedef QVector<pos>	poss;
typedef QVector<rpy>	rpys;
typedef QVector<pse>	pses;

class vec3_{
public:
	union{
		struct{double x, y, z;};
		double v[3];
	};
	double  norm      () const;
	vec3   unit      () const;
	vec3_& normalize ();
	vec3_& negate    ();
		  double& operator[] (int i)       {return v[i];}
	const double& operator[] (int i) const {return v[i];}
};
class vec4_{
public:
	union{
		struct{double x, y, z, p;};
		struct{vec3_ v3;};
		double v[4];
	};
	double  norm      () const;
	vec4   unit      () const;
	vec4_& normalize ();
	vec4_& negate    ();
		  double& operator[] (int i)       {return v[i];}
	const double& operator[] (int i) const {return v[i];}
};
class mat3_{
public:
	union{
		struct{double m00,m01,m02,m10,m11,m12,m20,m21,m22;};
		struct{vec3_ x, y, z;};
		double m[3][3];
		double v[9];
	};

	double  det() const;
	mat3_& negate();
	mat3_& transpose();
	mat3_& inverse();
		  double* operator[] (int i)       {return m[i];}
	const double* operator[] (int i) const {return m[i];}
		  double& operator() (int i, int j)       {return m[i][j];}
	const double& operator() (int i, int j) const {return m[i][j];}
};
class mat4_{
public:
	union{
		struct{double m00,m01,m02,m03,m10,m11,m12,m13,m20,m21,m22,m23,m30,m31,m32,m33;};
		struct{vec4_ x, y, z, p;};
		double m[4][4];
		double v[16];
	};
	double  det() const;
	mat4_& negate();
	mat4_& transpose();
	mat4_& inverse();
		  double* operator[] (int i)       {return m[i];}
	const double* operator[] (int i) const {return m[i];}
		  double& operator() (int i, int j)       {return m[i][j];}
	const double& operator() (int i, int j) const {return m[i][j];}
};
class quat_{
public:
	union{
		struct{double w, x, y, z;};
		struct{double wq; vec3_ r;};
		double v[4];
	};
		  double& operator[] (int i)       {return v[i];}
	const double& operator[] (int i) const {return v[i];}
};
class rpy_{
public:
	union{
		struct{double r,p,y;};
		double v[3];
	};
		  double& operator[] (int i)       {return v[i];}
	const double& operator[] (int i) const {return v[i];}
};
class pos_{
public:
	union{
		struct{vec3_ p; quat_ q;};
		struct{double x, y, z, qw, qx, qy, qz;};
		double v[7];
	};
		  double& operator[] (int i)       {return v[i];}
	const double& operator[] (int i) const {return v[i];}
};
class pse_{
public:
	union{
		struct{vec3_ p; rpy_ e;};
		struct{double x, y, z, er, ep, ey;};
		double v[6];
	};
		  double& operator[] (int i)       {return v[i];}
	const double& operator[] (int i) const {return v[i];}
};

class vec3 : public vec3_{
public:
	vec3(double val=0);
	vec3(double x, double y, double z);
	vec3(const vec3_& v);
	vec3(const vec4_& v);
	vec3(const rpy_&  v);
	vec3(const double* v);
};
class vec4 : public vec4_{
public:
	vec4(double val=0, double p=1);
	vec4(double x, double y, double z, double p=1);
	vec4(const vec3_& v, double p=1);
	vec4(const vec4_& v);
	vec4(const double* v);
};
class mat3 : public mat3_{
public:
	mat3(double val=0);
	mat3(double a00, double a01, double a02,
		 double a10, double a11, double a12,
		 double a20, double a21, double a22);
	mat3(const vec3_& axis, double angle);
	mat3(const mat3_& m);
	mat3(const mat4_& m);
	mat3(const quat_& q);
	mat3(const rpy_&  e);
	mat3(const double* v);
	inline static mat3 eye      ()                             {return mat3(1,0,0,0,1,0,0,0,1);}
	inline static mat3 diag     (const vec3& p)                {return mat3(p.x,0,0,0,p.y,0,0,0,p.z);}
	inline static mat3 diag     (double x, double y, double z)    {return mat3(x,0,0,0,y,0,0,0,z);}
	inline static mat3 rotate   (double angle, const vec3_& axis){
		double s=sin(angle),c=cos(angle);
		double l = axis.norm();
		double x=axis.x/l,y=axis.y/l,z=axis.z/l;
		return mat3(c + x*x*(1-c),   x*y*(1-c) - z*s,  x*z*(1-c) + y*s,
					y*x*(1-c) + z*s, c + y*y*(1-c),    y*z*(1-c) - x*s,
					z*x*(1-c) - y*s, z*y*(1-c) + x*s,  c + z*z*(1-c));
	}
	inline static mat3 rotate   (double wx, double wy, double wz) {
		double cx = cos(wx), cy = cos(wy), cz = cos(wz);
		double sx = sin(wx), sy = sin(wy), sz = sin(wz);
		return mat3(         cy*cz,         -cy*sz,     sy,
					cx*sz+cz*sx*sy, cx*cz-sx*sy*sz, -cy*sx,
					sx*sz-cx*cz*sy, cx*sy*sz+cz*sx,  cx*cy);
	}
	inline static mat3 rotateX  (double angle)                  {double s=sin(angle),c=cos(angle); return mat3(1,0,0,0,c,-s,0,s,c);}
	inline static mat3 rotateY  (double angle)                  {double s=sin(angle),c=cos(angle); return mat3(c,0,s,0,1,0,-s,0,c);}
	inline static mat3 rotateZ  (double angle)                  {double s=sin(angle),c=cos(angle); return mat3(c,-s,0,s,c,0,0,0,1);}
	inline static mat3 scale    (double s)                      {return mat3(s,0,0,0,s,0,0,0,s);}
	inline static mat3 scale    (const vec3_& s)               {return mat3(s.x,0,0,0,s.y,0,0,0,s.z);}
	inline static mat3 scale    (double x, double y, double z)    {return mat3(x,0,0,0,y,0,0,0,z);}
};
class mat4 : public mat4_{
public:
	mat4(double val=0);
	mat4(double a00, double a01, double a02,
		 double a10, double a11, double a12,
		 double a20, double a21, double a22);
	mat4(double a00, double a01, double a02, double a03,
		 double a10, double a11, double a12, double a13,
		 double a20, double a21, double a22, double a23);
	mat4(double a00, double a01, double a02, double a03,
		 double a10, double a11, double a12, double a13,
		 double a20, double a21, double a22, double a23,
		 double a30, double a31, double a32, double a33);
	mat4(const vec3_& axis, double angle);
	mat4(const vec3_& offset, const vec3_& axis, double angle);
	mat4(const mat3_& m);
	mat4(const vec3_& offset, const mat3_& m);
	mat4(const mat4_& m);
	mat4(const quat_& q);
	mat4(const rpy_&  e);
	mat4(const vec3_& offset, const quat_& q);
	mat4(const vec3_& x, const vec3_& y, const vec3_& z);
	mat4(const vec3_& x, const vec3_& y, const vec3_& z, const vec3_& p);
	mat4(const pos_& p);
	mat4(const double* v);
	inline static mat4 eye      ()                             {return mat4(1,0,0,0,1,0,0,0,1);}
	inline static mat4 diag     (const vec3& p)                {return mat4(p.x,0,0,0,p.y,0,0,0,p.z);}
	inline static mat4 diag     (double x, double y, double z)    {return mat4(x,0,0,0,y,0,0,0,z);}
	inline static mat4 translate(const vec3_& p)               {return mat4(1,0,0,p.x,0,1,0,p.y,0,0,1,p.z);}
	inline static mat4 translate(double x, double y, double z)    {return mat4(1,0,0,x,0,1,0,y,0,0,1,z);}
	inline static mat4 rotate   (double angle, const vec3_& axis){
		double s=sin(angle),c=cos(angle);
		double l = axis.norm();
		double x=axis.x/l,y=axis.y/l,z=axis.z/l;
		return mat4(c + x*x*(1-c),   x*y*(1-c) - z*s,  x*z*(1-c) + y*s,
					y*x*(1-c) + z*s, c + y*y*(1-c),    y*z*(1-c) - x*s,
					z*x*(1-c) - y*s, z*y*(1-c) + x*s,  c + z*z*(1-c));
	}
	inline static mat4 rotate   (double wx, double wy, double wz) {
		double cx = cos(wx), cy = cos(wy), cz = cos(wz);
		double sx = sin(wx), sy = sin(wy), sz = sin(wz);
		return mat4(         cy*cz,         -cy*sz,     sy,
					cx*sz+cz*sx*sy, cx*cz-sx*sy*sz, -cy*sx,
					sx*sz-cx*cz*sy, cx*sy*sz+cz*sx,  cx*cy);
	}
	inline static mat4 rotateX  (double angle)                  {double s=sin(angle),c=cos(angle); return mat4(1,0,0,0,c,-s,0,s,c);}
	inline static mat4 rotateY  (double angle)                  {double s=sin(angle),c=cos(angle); return mat4(c,0,s,0,1,0,-s,0,c);}
	inline static mat4 rotateZ  (double angle)                  {double s=sin(angle),c=cos(angle); return mat4(c,-s,0,s,c,0,0,0,1);}
	inline static mat4 scale    (double s)                      {return mat4(s,0,0,0,s,0,0,0,s);}
	inline static mat4 scale    (const vec3_& s)               {return mat4(s.x,0,0,0,s.y,0,0,0,s.z);}
	inline static mat4 scale    (double x, double y, double z)    {return mat4(x,0,0,0,y,0,0,0,z);}
};
class quat : public quat_{
public:
	quat();
	quat(double w, double x, double y, double z);
	quat(const quat_& q);
	quat(const rpy_&  e);
	quat(const mat3_& m);
	quat(const mat4_& m);
	quat(const vec3_& axis, double angle);
	quat(const double* v);
	inline static quat rotateX  (double angle)                  {return quat(cos(angle/2.f),sin(angle/2.f),0,0);}
	inline static quat rotateY  (double angle)                  {return quat(cos(angle/2.f),0,sin(angle/2.f),0);}
	inline static quat rotateZ  (double angle)                  {return quat(cos(angle/2.f),0,0,sin(angle/2.f));}
	inline static quat rotate   (double angle, const vec3_& axis){ double a = sin(angle/2.f)/axis.norm(); return quat(cos(angle/2.f), a*axis.x, a*axis.y, a*axis.z);}
};
class pos : public pos_{
public:
	pos();
	pos(double x, double y, double z);
	pos(double x, double y, double z, double qw, double qx, double qy, double qz);
	pos(const pos_& p);
	pos(const pse_& p);
	pos(const vec3_& p);
	pos(const vec3_& p, const quat_& q);
	pos(const vec4_& p);
	pos(const vec4_& p, const quat_& q);
	pos(const mat3& m);
	pos(const vec3_& offset, const mat3& m);
	pos(const mat4& m);
	pos(const vec3_& axis, double angle);
	pos(const vec3_& offset, const vec3_& axis, double angle);
	pos(const double* v);
};
class rpy : public rpy_{
public:
	rpy();
	rpy(double r, double p, double y);
	rpy(const rpy_& e);
	rpy(const quat_& q);
	rpy(const mat3_& m);
	rpy(const mat4_& m);
	rpy(const vec3_& rv);
	rpy(const vec3_& axis, double angle);
	rpy(const double* v);
	static rpy rotateX  (double angle);
	static rpy rotateY  (double angle);
	static rpy rotateZ  (double angle);
	static rpy rotate   (double angle, const vec3_& axis);
};
class pse : public pse_{
public:
	pse();
	pse(double x, double y, double z);
	pse(double x, double y, double z, double er, double ep, double ey);
	pse(const pse_& p);
	pse(const pos_& p);
	pse(const vec3_& p);
	pse(const vec3_& p, const quat_& q);
	pse(const vec4_& p);
	pse(const vec4_& p, const quat_& q);
	pse(const mat3& m);
	pse(const vec3_& offset, const mat3& m);
	pse(const mat4& m);
	pse(const vec3_& axis, double angle);
	pse(const vec3_& offset, const vec3_& axis, double angle);
	pse(const double* v);
};

// =======================================
//              constructors
// =======================================
inline vec3::vec3(double val)                         {this->x=val;this->y=val;this->z=val;}
inline vec3::vec3(double x, double y, double z)         {this->x=x;  this->y=y;  this->z=z;}
inline vec3::vec3(const vec3_& v)                    {this->x=v.x;this->y=v.y;this->z=v.z;}
inline vec3::vec3(const vec4_& v)                    {this->x=v.x;this->y=v.y;this->z=v.z;}
inline vec3::vec3(const rpy_&  v)                    {this->x=v.r;this->y=v.p;this->z=v.y;}
inline vec3::vec3(const double* v)                    {this->x=v[0];this->y=v[1];this->z=v[2];}

inline vec4::vec4(double val, double p)                {this->x=val;this->y=val;this->z=val;this->p=p;}
inline vec4::vec4(double x, double y, double z, double p){this->x=x;  this->y=y;  this->z=z;  this->p=p;}
inline vec4::vec4(const vec3_& v, double p)           {this->x=v.x;this->y=v.y;this->z=v.z;this->p=p;}
inline vec4::vec4(const vec4_& v)                    {this->x=v.x;this->y=v.y;this->z=v.z;this->p=v.p;}
inline vec4::vec4(const double* v)                    {this->x=v[0];this->y=v[1];this->z=v[2];this->p=v[3];}

inline mat3::mat3(double val){
	m00 = val; m01 = val; m02 = val;
	m10 = val; m11 = val; m12 = val;
	m20 = val; m21 = val; m22 = val;
}
inline mat3::mat3(double a00, double a01, double a02,
		   double a10, double a11, double a12,
		   double a20, double a21, double a22){
	m00 = a00; m01 = a01; m02 = a02;
	m10 = a10; m11 = a11; m12 = a12;
	m20 = a20; m21 = a21; m22 = a22;
}
inline mat3::mat3(const vec3_& axis, double angle){
	double c = cos(angle), s = sin(angle);
	double l = axis.norm();
	double x = axis.x/l, y = axis.y/l, z = axis.z/l;
	m00 = c + x*x*(1-c);   m10 = x*y*(1-c) - z*s;  m20 = x*z*(1-c) + y*s;
	m01 = y*x*(1-c) + z*s; m11 = c + y*y*(1-c);    m21 = y*z*(1-c) - x*s;
	m02 = z*x*(1-c) - y*s; m12 = z*y*(1-c) + x*s;  m22 = c + z*z*(1-c);

}
inline mat3::mat3(const mat3_& m){
	m00 = m.m00; m01 = m.m01; m02 = m.m02;
	m10 = m.m10; m11 = m.m11; m12 = m.m12;
	m20 = m.m20; m21 = m.m21; m22 = m.m22;
}
inline mat3::mat3(const mat4_& m){
	m00 = m.m00; m01 = m.m01; m02 = m.m02;
	m10 = m.m10; m11 = m.m11; m12 = m.m12;
	m20 = m.m20; m21 = m.m21; m22 = m.m22;
}
inline mat3::mat3(const double* v)                    {memcpy(this->v, v, sizeof(mat3));}
inline mat3::mat3(const quat_& q){
	double xx = 2*q.x*q.x, yy = 2*q.y*q.y, zz = 2*q.z*q.z;
	double wx = 2*q.w*q.x, wy = 2*q.w*q.y, wz = 2*q.w*q.z;
	double xy = 2*q.x*q.y, yz = 2*q.y*q.z, zx = 2*q.z*q.x;

	m00 = 1.0f-yy-zz; m01 =      xy-wz; m02 =      zx+wy;
	m10 =      xy+wz; m11 = 1.0f-xx-zz; m12 =      yz-wx;
	m20 =      zx-wy; m21 =      yz+wx; m22 = 1.0f-xx-yy;
}
inline mat3::mat3(const rpy_& e){
	double cx = cos(e.r), cy = cos(e.p), cz = cos(e.y);
	double sx = sin(e.r), sy = sin(e.p), sz = sin(e.y);

	m00 = cy*cz; m01 = -cx*sz + sx*sy*cz; m02 =  sx*sz + cx*sy*cz;
	m10 = cy*sz; m11 =  cx*cz + sx*sy*sz; m12 = -sx*cz + cx*sy*sz;
	m20 =   -sy; m21 =             sx*cy; m22 =             cx*cy;
}
inline mat4::mat4(double val){
	m00 = val; m01 = val; m02 = val; m03 = val;
	m10 = val; m11 = val; m12 = val; m13 = val;
	m20 = val; m21 = val; m22 = val; m23 = val;
	m30 = val; m31 = val; m32 = val; m33 = val;
}
inline mat4::mat4(double a00, double a01, double a02,
		   double a10, double a11, double a12,
		   double a20, double a21, double a22){
	m00 = a00; m01 = a01; m02 = a02; m03 = 0;
	m10 = a10; m11 = a11; m12 = a12; m13 = 0;
	m20 = a20; m21 = a21; m22 = a22; m23 = 0;
	m30 =   0; m31 =   0; m32 =   0; m33 = 1;
}
inline mat4::mat4(double a00, double a01, double a02, double a03,
		   double a10, double a11, double a12, double a13,
		   double a20, double a21, double a22, double a23){
	m00 = a00; m01 = a01; m02 = a02; m03 = a03;
	m10 = a10; m11 = a11; m12 = a12; m13 = a13;
	m20 = a20; m21 = a21; m22 = a22; m23 = a23;
	m30 =   0; m31 =   0; m32 =   0; m33 = 1;
}
inline mat4::mat4(double a00, double a01, double a02, double a03,
		   double a10, double a11, double a12, double a13,
		   double a20, double a21, double a22, double a23,
		   double a30, double a31, double a32, double a33){
	m00 = a00; m01 = a01; m02 = a02; m03 = a03;
	m10 = a10; m11 = a11; m12 = a12; m13 = a13;
	m20 = a20; m21 = a21; m22 = a22; m23 = a23;
	m30 = a30; m31 = a31; m32 = a32; m33 = a33;
}
inline mat4::mat4(const vec3_& axis, double angle){
	double c = cos(angle), s = sin(angle);
	double C = 1-c;
	double l = axis.norm();
	double x = axis.x/l, y = axis.y/l, z = axis.z/l;
	m00 =   c + x*x*C; m01 = x*y*C - z*s; m02 = x*z*C + y*s; m03 = 0;
	m10 = y*x*C + z*s; m11 =   c + y*y*C; m12 = y*z*C - x*s; m13 = 0;
	m20 = z*x*C - y*s; m21 = z*y*C + x*s; m22 =   c + z*z*C; m23 = 0;
	m30 =           0; m31 =           0; m32 =           0; m33 = 1;
}
inline mat4::mat4(const vec3_& offset, const vec3_& axis, double angle){
	double c = cos(angle), s = sin(angle);
	double C = 1-c;
	double l = axis.norm();
	double x = axis.x/l, y = axis.y/l, z = axis.z/l;
	m00 =   c + x*x*C; m01 = x*y*C - z*s; m02 = x*z*C + y*s; m03 = offset.x;
	m10 = y*x*C + z*s; m11 =   c + y*y*C; m12 = y*z*C - x*s; m13 = offset.y;
	m20 = z*x*C - y*s; m21 = z*y*C + x*s; m22 =   c + z*z*C; m23 = offset.z;
	m30 =           0; m31 =           0; m32 =           0; m33 = 1;
}
inline mat4::mat4(const mat3_& m){
	m00 = m.m00; m01 = m.m01; m02 = m.m02; m03 = 0;
	m10 = m.m10; m11 = m.m11; m12 = m.m12; m13 = 0;
	m20 = m.m20; m21 = m.m21; m22 = m.m22; m23 = 0;
	m30 =     0; m31 =     0; m32 =     0; m33 = 1;
}
inline mat4::mat4(const mat4_& m){
	m00 = m.m00; m01 = m.m01; m02 = m.m02; m03 = m.m03;
	m10 = m.m10; m11 = m.m11; m12 = m.m12; m13 = m.m13;
	m20 = m.m20; m21 = m.m21; m22 = m.m22; m23 = m.m23;
	m30 = m.m30; m31 = m.m31; m32 = m.m32; m33 = m.m33;
}
inline mat4::mat4(const quat_& q){
	double xx = 2*q.x*q.x, yy = 2*q.y*q.y, zz = 2*q.z*q.z;
	double wx = 2*q.w*q.x, wy = 2*q.w*q.y, wz = 2*q.w*q.z;
	double xy = 2*q.x*q.y, yz = 2*q.y*q.z, zx = 2*q.z*q.x;

	m00 = 1.0f-yy-zz; m01 =      xy-wz; m02 =      zx+wy; m03 = 0;
	m10 =      xy+wz; m11 = 1.0f-xx-zz; m12 =      yz-wx; m13 = 0;
	m20 =      zx-wy; m21 =      yz+wx; m22 = 1.0f-xx-yy; m23 = 0;
	m30 =          0; m31 =          0; m32 =          0; m33 = 1;
}
inline mat4::mat4(const rpy_& e){
	double cx = cos(e.r), cy = cos(e.p), cz = cos(e.y);
	double sx = sin(e.r), sy = sin(e.p), sz = sin(e.y);

	m00 = cy*cz; m01 = -cx*sz + sx*sy*cz; m02 =  sx*sz + cx*sy*cz; m03 = 0;
	m10 = cy*sz; m11 =  cx*cz + sx*sy*sz; m12 = -sx*cz + cx*sy*sz; m13 = 0;
	m20 =   -sy; m21 =             sx*cy; m22 =             cx*cy; m23 = 0;
	m30 =     0; m31 =                 0; m32 =                 0; m33 = 1;
}
inline mat4::mat4(const vec3_& offset, const quat_& q){
	double xx = 2*q.x*q.x, yy = 2*q.y*q.y, zz = 2*q.z*q.z;
	double wx = 2*q.w*q.x, wy = 2*q.w*q.y, wz = 2*q.w*q.z;
	double xy = 2*q.x*q.y, yz = 2*q.y*q.z, zx = 2*q.z*q.x;

	m00 = 1.0f-yy-zz; m01 =      xy-wz; m02 =      zx+wy; m03 = offset.x;
	m10 =      xy+wz; m11 = 1.0f-xx-zz; m12 =      yz-wx; m13 = offset.y;
	m20 =      zx-wy; m21 =      yz+wx; m22 = 1.0f-xx-yy; m23 = offset.z;
	m30 =          0; m31 =          0; m32 =          0; m33 = 1;
}
inline mat4::mat4(const vec3_& x, const vec3_& y, const vec3_& z){
	m00 = x.x; m01 = y.x; m02 = z.x; m03 = 0;
	m10 = x.y; m11 = y.y; m12 = z.y; m13 = 0;
	m20 = x.z; m21 = y.z; m22 = z.z; m23 = 0;
	m30 =   0; m31 =   0; m32 =   0; m33 = 1;
}
inline mat4::mat4(const vec3_& x, const vec3_& y, const vec3_& z, const vec3_& p){
	m00 = x.x; m01 = y.x; m02 = z.x; m03 = p.x;
	m10 = x.y; m11 = y.y; m12 = z.y; m13 = p.y;
	m20 = x.z; m21 = y.z; m22 = z.z; m23 = p.z;
	m30 =   0; m31 =   0; m32 =   0; m33 = 1;
}
inline mat4::mat4(const pos_& p){
	double xx = 2*p.qx*p.qx, yy = 2*p.qy*p.qy, zz = 2*p.qz*p.qz;
	double wx = 2*p.qw*p.qx, wy = 2*p.qw*p.qy, wz = 2*p.qw*p.qz;
	double xy = 2*p.qx*p.qy, yz = 2*p.qy*p.qz, zx = 2*p.qz*p.qx;

	m00 = 1.0f-yy-zz; m01 =      xy-wz; m02 =      zx+wy; m03 = p.x;
	m10 =      xy+wz; m11 = 1.0f-xx-zz; m12 =      yz-wx; m13 = p.y;
	m20 =      zx-wy; m21 =      yz+wx; m22 = 1.0f-xx-yy; m23 = p.z;
	m30 =          0; m31 =          0; m32 =          0; m33 =   1;
}
inline mat4::mat4(const double* v)                    {memcpy(this->v, v, sizeof(mat4));}

inline quat::quat()                                  {this->w = 1; this->x = 0; this->y = 0; this->z = 0;}
inline quat::quat(double w, double x, double y, double z){this->w = w; this->x = x; this->y = y; this->z = z;}
inline quat::quat(const quat_& q)                    {this->w =q.w; this->x =q.x; this->y =q.y; this->z =q.z;}
inline quat::quat(const rpy_&  e)                    {
	double cr = cos(e.r/2), cp = cos(e.p/2), cy = cos(e.y/2);
	double sr = sin(e.r/2), sp = sin(e.p/2), sy = sin(e.y/2);
	w = cr*cp*cy + sr*sp*sy;
	x = sr*cp*cy - cr*sp*sy;
	y = cr*sp*cy + sr*cp*sy;
	z = cr*cp*sy - sr*sp*cy;
}
inline quat::quat(const mat3_& m){
    w = 0.5f*sqrtp(1.0f+m.m00+m.m11+m.m22);
	x = sign(m.m21-m.m12)*0.5f*sqrtp(1+m.m00-m.m11-m.m22);
	y = sign(m.m02-m.m20)*0.5f*sqrtp(1-m.m00+m.m11-m.m22);
	z = sign(m.m10-m.m01)*0.5f*sqrtp(1-m.m00-m.m11+m.m22);
}
inline quat::quat(const mat4_& m){
	w = 0.5f*sqrtp(1.0f+m.m00+m.m11+m.m22);
	x = sign(m.m21-m.m12)*0.5f*sqrtp(1+m.m00-m.m11-m.m22);
	y = sign(m.m02-m.m20)*0.5f*sqrtp(1-m.m00+m.m11-m.m22);
	z = sign(m.m10-m.m01)*0.5f*sqrtp(1-m.m00-m.m11+m.m22);
}
inline quat::quat(const vec3_& axis, double angle){
	double s = sin(angle/2);
	vec3 a = axis.unit();
	w = cos(angle/2);
	x = s*a.x;
	y = s*a.y;
	z = s*a.z;
}
inline quat::quat(const double* v)                    {memcpy(this->v, v, sizeof(quat));}

inline pos::pos()                                                                 {this->x=0; this->y=0; this->z=0; this->qw=1; this->qx=0; this->qy=0; this->qz=0;}
inline pos::pos(double x, double y, double z)                                        {this->x=x; this->y=y; this->z=z; this->qw=1; this->qx=0; this->qy=0; this->qz=0;}
inline pos::pos(double x, double y, double z, double qw, double qx, double qy, double qz){this->x=x; this->y=y; this->z=z; this->qw=qw; this->qx=qx; this->qy=qy; this->qz=qz;}
inline pos::pos(const pos_& p)                                                    {this->x=p.x; this->y=p.y; this->z=p.z; this->qw=p.qw; this->qx=p.qx; this->qy=p.qy; this->qz=p.qz;}
inline pos::pos(const pse_& p)                                                    {this->p = p.p; this->q = quat(p.e);}
inline pos::pos(const vec3_& p)                                                   {this->p=p; this->qw=1; this->qx=0; this->qy=0; this->qz=0;}
inline pos::pos(const vec3_& p, const quat_& q)                                   {this->p=p; this->q=q;}
inline pos::pos(const vec4_& p)                                                   {this->p=p.v3; this->qw=1; this->qx=0; this->qy=0; this->qz=0;}
inline pos::pos(const vec4_& p, const quat_& q)                                   {this->p=p.v3; this->q=q;}
inline pos::pos(const double* v)                                                   {memcpy(this->v, v, sizeof(pos));}
inline pos::pos(const mat3& m){
	x  = 0;
	y  = 0;
	z  = 0;
	qw = 0.5f*sqrtp(1.0f+m.m00+m.m11+m.m22);
	qx = sign(m.m21-m.m12)*0.5f*sqrtp(1.0f+m.m00-m.m11-m.m22);
	qy = sign(m.m02-m.m20)*0.5f*sqrtp(1.0f-m.m00+m.m11-m.m22);
	qz = sign(m.m10-m.m01)*0.5f*sqrtp(1.0f-m.m00-m.m11+m.m22);
}
inline pos::pos(const vec3_& offset, const mat3& m){
	p = offset;
    qw = 0.5f*sqrtp(1.0f+m.m00+m.m11+m.m22);
	qx = sign(m.m21-m.m12)*0.5f*sqrtp(1.0f+m.m00-m.m11-m.m22);
	qy = sign(m.m02-m.m20)*0.5f*sqrtp(1.0f-m.m00+m.m11-m.m22);
	qz = sign(m.m10-m.m01)*0.5f*sqrtp(1.0f-m.m00-m.m11+m.m22);
}
inline pos::pos(const mat4& m){
	x  = m.m03;
	y  = m.m13;
	z  = m.m23;
	qw = 0.5f*sqrtp(1.0f+m.m00+m.m11+m.m22);
	qx = sign(m.m21-m.m12)*0.5f*sqrtp(1.0f+m.m00-m.m11-m.m22);
	qy = sign(m.m02-m.m20)*0.5f*sqrtp(1.0f-m.m00+m.m11-m.m22);
	qz = sign(m.m10-m.m01)*0.5f*sqrtp(1.0f-m.m00-m.m11+m.m22);
}
inline pos::pos(const vec3_& axis, double angle){
	double s = sin(angle/2);
	vec3 a = axis.unit();
	q.w = cos(angle/2);
	q.x = s*a.x;
	q.y = s*a.y;
	q.z = s*a.z;
}
inline pos::pos(const vec3_& offset, const vec3_& axis, double angle){
	p = offset;
	double s = sin(angle/2);
	vec3 a = axis.unit();
	q.w = cos(angle/2);
	q.x = s*a.x;
	q.y = s*a.y;
	q.z = s*a.z;
}


inline rpy::rpy()                                                       {this->r = 0;  this->p = 0; this->y = 0;}
inline rpy::rpy(double r, double p, double y)                              {this->r = r;  this->p = p; this->y = y;}
inline rpy::rpy(const rpy_& e)                                          {this->r =e.r;  this->p =e.p; this->y =e.y;}
inline rpy::rpy(const quat_& q){
	double l = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
	double w, x, y, z;
	if(l > 0.00001){
        l = sqrtp(l);
		w = q.w/l; x = q.x/l; y = q.y/l; z = q.z/l;
	}else{
		w = 1; x = 0; y = 0; z = 0;
	}
	this->r = atan2(2*(w*x + y*z), 1-2*(x*x + y*y));
	this->p = asin(2*(w*y - z*x));
	this->y = atan2(2*(w*z + x*y), 1-2*(y*y + z*z));
}
//	inline rpy::rpy(const mat3_& m);
//	inline rpy::rpy(const mat4_& m);
inline rpy::rpy(const vec3_& rv)                        {this->r = rv.x; this->p = rv.y; this->y = rv.z;}
//	inline rpy::rpy(const vec3_& axis, double angle);
inline rpy::rpy(const double* v)                         {memcpy(this->v, v, sizeof(rpy));}
inline rpy rpy::rotateX  (double angle)                  {return rpy(angle,0,0);}
inline rpy rpy::rotateY  (double angle)                  {return rpy(0,angle,0);}
inline rpy rpy::rotateZ  (double angle)                  {return rpy(0,0,angle);}
//	inline rpy rpy::rotate   (double angle, const vec3_& axis){mat3::}

inline pse::pse()                                                       {this->x = 0; this->y = 0; this->z = 0; this->er =  0; this->ep =  0; this->ey =  0;}
inline pse::pse(double x, double y, double z)                              {this->x = x; this->y = y; this->z = z; this->er =  0; this->ep =  0; this->ey =  0;}
inline pse::pse(double x, double y, double z, double er, double ep, double ey){this->x = x; this->y = y; this->z = z; this->er = er; this->ep = ep; this->ey = ey;}
inline pse::pse(const pse_& p)                                          {this->x =p.x; this->y =p.y; this->z =p.z; this->er =p.er; this->ep =p.ep; this->ey =p.ey;}
inline pse::pse(const pos_& p)                                          {this->p = p.p; this->e = rpy(p.q);}
//	inline pse::pse(const vec3_& p);
//	inline pse::pse(const vec3_& p, const quat_& q);
//	inline pse::pse(const vec4_& p);
//	inline pse::pse(const vec4_& p, const quat_& q);
//	inline pse::pse(const mat3& m);
//	inline pse::pse(const vec3_& offset, const mat3& m);
//	inline pse::pse(const mat4& m);
//	inline pse::pse(const vec3_& axis, double angle);
//	inline pse::pse(const vec3_& offset, const vec3_& axis, double angle);
inline pse::pse(const double* v)                         {memcpy(this->v, v, sizeof(pse));}

// =======================================
//      member functions
// =======================================
inline double  vec3_::norm      ()                  const {return sqrtp(x*x+y*y+z*z);}
inline vec3   vec3_::unit      ()                  const {double div = 1.0f/this->norm(); return vec3(x*div, y*div, z*div);}
inline vec3_& vec3_::normalize ()                        {double div = 1.0f/this->norm(); x*=div; y*=div; z*=div; return *this;}
inline vec3_& vec3_::negate    ()                        {x = -x; y = -y; z = -z; return *this;}
inline double  vec4_::norm      ()                  const {return sqrtp(x*x+y*y+z*z);}
inline vec4   vec4_::unit      ()                  const {double div = 1.0f/this->norm(); return vec4(x*div, y*div, z*div,p);}
inline vec4_& vec4_::normalize ()                        {double div = 1.0f/this->norm(); x*=div; y*=div; z*=div; return *this;}
inline vec4_& vec4_::negate    ()                        {x = -x; y = -y; z = -z; return *this;}
inline double   mat3_::det      ()                  const {return (m01*m12-m02*m11)*m20 + (m02*m10-m00*m12)*m21 + (m00*m11-m01*m10)*m22;}
inline mat3_&  mat3_::inverse  ()                        {
	double div = 1.0f/det();
	double b00 = - (m12*m21-m11*m22)*div;
	double b01 =   (m02*m21-m01*m22)*div;
	double b02 = - (m02*m11-m01*m12)*div;
	double b10 =   (m12*m20-m10*m22)*div;
	double b11 = - (m02*m20-m00*m22)*div;
	double b12 =   (m02*m10-m00*m12)*div;
	double b20 = - (m11*m20-m10*m21)*div;
	double b21 =   (m01*m20-m00*m21)*div;
	double b22 = - (m01*m10-m00*m11)*div;
	m00 = b00; m01 = b01; m02 = b02;
	m10 = b10; m11 = b11; m12 = b12;
	m20 = b20; m21 = b21; m22 = b22;
	return *this;
}
inline mat3_&  mat3_::negate   ()                        {
	m00 = -m00; m01 = -m01; m02 = -m02;
	m10 = -m10; m11 = -m11; m12 = -m12;
	m20 = -m20; m21 = -m21; m22 = -m22;
	return *this;
}
inline mat3_&  mat3_::transpose()                        {
	swap(m01, m10); swap(m02, m20); swap(m12, m21);
	return *this;
}
inline double   mat4_::det      ()                  const {
	return ( (m03*m12-m02*m13)*m21 + (m01*m13-m03*m11)*m22 + (m02*m11-m01*m12)*m23 )*m30
		 + ( (m02*m13-m03*m12)*m20 + (m03*m10-m00*m13)*m22 + (m00*m12-m02*m10)*m23 )*m31
		 + ( (m03*m11-m01*m13)*m20 + (m00*m13-m03*m10)*m21 + (m01*m10-m00*m11)*m23 )*m32
		 + ( (m01*m12-m02*m11)*m20 + (m02*m10-m00*m12)*m21 + (m00*m11-m01*m10)*m22 )*m33;
}
inline mat4_&  mat4_::inverse  ()                        {
	double div = 1.0f/det();
	double b00 = ((m12*m23-m13*m22)*m31 + (m13*m21-m11*m23)*m32 - (m12*m21-m11*m22)*m33)*div;
	double b01 = ((m03*m22-m02*m23)*m31 - (m03*m21-m01*m23)*m32 + (m02*m21-m01*m22)*m33)*div;
	double b02 = ((m02*m13-m03*m12)*m31 + (m03*m11-m01*m13)*m32 - (m02*m11-m01*m12)*m33)*div;
	double b03 = ((m03*m12-m02*m13)*m21 - (m03*m11-m01*m13)*m22 + (m02*m11-m01*m12)*m23)*div;
	double b10 = ((m13*m22-m12*m23)*m30 - (m13*m20-m10*m23)*m32 + (m12*m20-m10*m22)*m33)*div;
	double b11 = ((m02*m23-m03*m22)*m30 + (m03*m20-m00*m23)*m32 - (m02*m20-m00*m22)*m33)*div;
	double b12 = ((m03*m12-m02*m13)*m30 - (m03*m10-m00*m13)*m32 + (m02*m10-m00*m12)*m33)*div;
	double b13 = ((m02*m13-m03*m12)*m20 + (m03*m10-m00*m13)*m22 - (m02*m10-m00*m12)*m23)*div;
	double b20 = ((m11*m23-m13*m21)*m30 + (m13*m20-m10*m23)*m31 - (m11*m20-m10*m21)*m33)*div;
	double b21 = ((m03*m21-m01*m23)*m30 - (m03*m20-m00*m23)*m31 + (m01*m20-m00*m21)*m33)*div;
	double b22 = ((m01*m13-m03*m11)*m30 + (m03*m10-m00*m13)*m31 - (m01*m10-m00*m11)*m33)*div;
	double b23 = ((m03*m11-m01*m13)*m20 - (m03*m10-m00*m13)*m21 + (m01*m10-m00*m11)*m23)*div;
	double b30 = ((m12*m21-m11*m22)*m30 - (m12*m20-m10*m22)*m31 + (m11*m20-m10*m21)*m32)*div;
	double b31 = ((m01*m22-m02*m21)*m30 + (m02*m20-m00*m22)*m31 - (m01*m20-m00*m21)*m32)*div;
	double b32 = ((m02*m11-m01*m12)*m30 - (m02*m10-m00*m12)*m31 + (m01*m10-m00*m11)*m32)*div;
	double b33 = ((m01*m12-m02*m11)*m20 + (m02*m10-m00*m12)*m21 - (m01*m10-m00*m11)*m22)*div;
	m00 = b00; m01 = b01; m02 = b02; m03 = b03; m10 = b10; m11 = b11; m12 = b12; m13 = b13;
	m20 = b20; m21 = b21; m22 = b22; m23 = b23; m30 = b30; m31 = b31; m32 = b32; m33 = b33;
	return *this;
}
inline mat4_&  mat4_::negate   ()                        {
	m00 = -m00; m01 = -m01; m02 = -m02; m03 = -m03;
	m10 = -m10; m11 = -m11; m12 = -m12; m13 = -m13;
	m20 = -m20; m21 = -m21; m22 = -m22; m23 = -m23;
	m30 = -m30; m31 = -m31; m32 = -m32; m33 = -m33;
	return *this;
}
inline mat4_&  mat4_::transpose()                        {
	swap(m01, m10); swap(m02, m20); swap(m03, m30);
	swap(m12, m21); swap(m13, m31); swap(m23, m32);
	return *this;
}
// =======================================
//      misc functions
// =======================================
inline void axisAngle(const quat_& q, double& angle, vec3& axis){
	if(q.r.norm() < 0.0001){
		angle = 0;
		axis = vec3(1,0,0);
	}else{
		angle = 2.f*acosf(q.w);
		axis = q.r.unit();
	}
}
// =======================================
//      vector_operators & functions
// =======================================
// vec3
inline double  norm      (const vec3_& v)                  {return sqrtp(v.x*v.x+v.y*v.y+v.z*v.z);}
inline vec3   unit      (const vec3_& v)                  {double div = 1.0f/norm(v); return vec3(v.x*div, v.y*div, v.z*div);}
inline vec3_& normalize (vec3_& v)                        {double div = 1.0f/norm(v); v.x*=div; v.y*=div; v.z*=div; return v;}
inline vec3_& negate    (vec3_& v)                        {v.x = -v.x; v.y = -v.y; v.z = -v.z; return v;}
inline vec3   operator -(const vec3_& v)                  {return vec3(-v.x,-v.y,-v.z);}
// vec4
inline double  norm      (const vec4_& v)                  {return sqrtp(v.x*v.x+v.y*v.y+v.z*v.z);}
inline vec4   unit      (const vec4_& v)                  {double div = 1.0f/norm(v); return vec4(v.x*div, v.y*div, v.z*div, v.p);}
inline vec4_& normalize (vec4_& v)                        {double div = 1.0f/norm(v); v.x*=div; v.y*=div; v.z*=div; return v;}
inline vec4_& negate    (vec4_& v)                        {v.x=-v.x; v.y=-v.y; v.z=-v.z; return v;}
inline vec4   operator -(const vec4_& v)                  {return vec4(-v.x,-v.y,-v.z,v.p);}
// quat
inline double  norm      (const quat_& q)                  {return sqrtp(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z);}
inline quat   unit      (const quat_& q)                  {double div = 1.0f/norm(q); return quat(q.w*div, q.x*div, q.y*div, q.z*div);}
inline quat_& negate    (quat_& q)                        {q.w=-q.w; q.x=-q.x; q.y=-q.y; q.z=-q.z; return q;}
inline quat_& inverse   (quat_& q)                        {q.x=-q.x; q.y=-q.y; q.z=-q.z; return q;}
inline quat   operator -(const quat_& q)                  {return quat(q.w,-q.x,-q.y,-q.z);}
inline quat   operator !(const quat_& q)                  {return quat(q.w,-q.x,-q.y,-q.z);}
// pos
inline pos_&  negate    (pos_& v)                         {negate(v.p); negate(v.q); return v;}
inline pos_&  inverse   (pos_& p);
inline pos    operator -(const pos_& v)                   {return pos(-v.x,-v.y,-v.z,v.qw,-v.qx,-v.qy,-v.qz);}
inline pos    operator !(const pos_& p)                   {pos ret = p; inverse(ret); return ret;}
// =======================================
//   vector-double_operators & functions
// =======================================
// vec3
inline vec3   operator + (double val, const vec3_& v)      {return vec3(val+v.x, val+v.y, val+v.z);}
inline vec3   operator - (double val, const vec3_& v)      {return vec3(val-v.x, val-v.y, val-v.z);}
inline vec3   operator * (double val, const vec3_& v)      {return vec3(val*v.x, val*v.y, val*v.z);}
inline vec3   operator / (double val, const vec3_& v)      {return vec3(val/v.x, val/v.y, val/v.z);}
inline vec3   operator + (const vec3_& v, double val)      {return vec3(v.x+val, v.y+val, v.z+val);}
inline vec3   operator - (const vec3_& v, double val)      {return vec3(v.x-val, v.y-val, v.z-val);}
inline vec3   operator * (const vec3_& v, double val)      {return vec3(v.x*val, v.y*val, v.z*val);}
inline vec3   operator / (const vec3_& v, double val)      {return vec3(v.x/val, v.y/val, v.z/val);}
inline vec3_& operator +=(vec3_& v, double val)            {v.x+=val; v.y+=val; v.z+=val; return v;}
inline vec3_& operator -=(vec3_& v, double val)            {v.x-=val; v.y-=val; v.z-=val; return v;}
inline vec3_& operator *=(vec3_& v, double val)            {v.x*=val; v.y*=val; v.z*=val; return v;}
inline vec3_& operator /=(vec3_& v, double val)            {v.x/=val; v.y/=val; v.z/=val; return v;}
// vec4
inline vec4   operator + (double val, const vec4_& v)      {return vec4(val+v.x, val+v.y, val+v.z, v.p);}
inline vec4   operator - (double val, const vec4_& v)      {return vec4(val-v.x, val-v.y, val-v.z, v.p);}
inline vec4   operator * (double val, const vec4_& v)      {return vec4(val*v.x, val*v.y, val*v.z, v.p);}
inline vec4   operator / (double val, const vec4_& v)      {return vec4(val/v.x, val/v.y, val/v.z, v.p);}
inline vec4   operator + (const vec4_& v, double val)      {return vec4(v.x+val, v.y+val, v.z+val, v.p);}
inline vec4   operator - (const vec4_& v, double val)      {return vec4(v.x-val, v.y-val, v.z-val, v.p);}
inline vec4   operator * (const vec4_& v, double val)      {return vec4(v.x*val, v.y*val, v.z*val, v.p);}
inline vec4   operator / (const vec4_& v, double val)      {return vec4(v.x/val, v.y/val, v.z/val, v.p);}
inline vec4_& operator +=(vec4_& v, double val)            {v.x+=val; v.y+=val; v.z+=val; return v;}
inline vec4_& operator -=(vec4_& v, double val)            {v.x-=val; v.y-=val; v.z-=val; return v;}
inline vec4_& operator *=(vec4_& v, double val)            {v.x*=val; v.y*=val; v.z*=val; return v;}
inline vec4_& operator /=(vec4_& v, double val)            {v.x/=val; v.y/=val; v.z/=val; return v;}
// =======================================
//   vector-vector_operators & functions
// =======================================
// vec3
inline vec3  operator +(const vec3_& v1, const vec3_& v2){return vec3(v1.x+v2.x, v1.y+v2.y, v1.z+v2.z);}
inline vec3  operator -(const vec3_& v1, const vec3_& v2){return vec3(v1.x-v2.x, v1.y-v2.y, v1.z-v2.z);}
inline vec3  cross     (const vec3_& v1, const vec3_& v2){return vec3(v1.y*v2.z - v1.z*v2.y,v1.z*v2.x - v1.x*v2.z,v1.x*v2.y - v1.y*v2.x);}
inline double dot       (const vec3_& v1, const vec3_& v2){return v1.x*v2.x+v1.y*v2.y+v1.z*v2.z;}
inline double dot       (const vec3_&  v)                 {return v.x*v.x+v.y*v.y+v.z*v.z;}
inline double dist      (const vec3_& v1, const vec3_& v2){return norm(v2-v1);}
inline vec3  proj      (const vec3_&  n, const vec3_&  x){return x-(dot(x,n)/dot(n))*n;}
// vec4
inline vec4  operator +(const vec4_& v1, const vec4_& v2){return vec4(v1.x+v2.x, v1.y+v2.y, v1.z+v2.z, v1.p||v2.p);}
inline vec4  operator -(const vec4_& v1, const vec4_& v2){return vec4(v1.x-v2.x, v1.y-v2.y, v1.z-v2.z, v1.p||v2.p);}
inline vec4  cross     (const vec4_& v1, const vec4_& v2){return vec4(v1.y*v2.z - v1.z*v2.y,v1.z*v2.x - v1.x*v2.z,v1.x*v2.y - v1.y*v2.x,0);}
inline double dot       (const vec4_& v1, const vec4_& v2){return v1.x*v2.x+v1.y*v2.y+v1.z*v2.z;}
inline double dot       (const vec4_&  v)                 {return v.x*v.x+v.y*v.y+v.z*v.z;}
inline double dist      (const vec4_& v1, const vec4_& v2){return norm(v2-v1);}
inline vec4  proj      (const vec4_&  n, const vec4_&  x){return x-(dot(x,n)/dot(n))*n;}
// quat
inline double dot       (const quat_& q1, const quat_& q2){return q1.w*q2.w+q1.x*q2.x+q1.y*q2.y+q1.z*q2.z;}
inline quat  operator *(const quat_& q1, const quat_& q2){
				//r = [p1.*q1 - p2.*q2 - p3.*q3 - p4.*q4;
				//	 p1.*q2 + p2.*q1 + p3.*q4 - p4.*q3;
				//	 p1.*q3 - p2.*q4 + p3.*q1 + p4.*q2;
				//	 p1.*q4 + p2.*q3 - p3.*q2 + p4.*q1];
	return quat(q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z,
				q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y,
				q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x,
				q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w);
}
inline quat  operator /(const quat_& q1, const quat_& q2){
	return q1*(-q2);
}
inline vec3  operator *(const quat_& q,  const vec3_& x){
	double pw =-q.x*x.x - q.y*x.y - q.z*x.z;
	double px = q.w*x.x + q.y*x.z - q.z*x.y;
	double py = q.w*x.y - q.x*x.z + q.z*x.x;
	double pz = q.w*x.z + q.x*x.y - q.y*x.x;
	return vec3(-pw*q.x + px*q.w - py*q.z + pz*q.y,
				-pw*q.y + px*q.z + py*q.w - pz*q.x,
				-pw*q.z - px*q.y + py*q.x + pz*q.w);
}
inline vec4  operator *(const quat_& q,  const vec4_& x){
	double pw =-q.x*x.x - q.y*x.y - q.z*x.z;
	double px = q.w*x.x + q.y*x.z - q.z*x.y;
	double py = q.w*x.y - q.x*x.z + q.z*x.x;
	double pz = q.w*x.z + q.x*x.y - q.y*x.x;
	return vec4(-pw*q.x + px*q.w - py*q.z + pz*q.y,
				-pw*q.y + px*q.z + py*q.w - pz*q.x,
				-pw*q.z - px*q.y + py*q.x + pz*q.w,x.p);
}
// pos
inline pos   operator *(const pos_&  p1, const pos_&  p2){return pos(p1.p + p1.q*p2.p, p1.q*p2.q);}
inline pos   operator /(const pos_&  p1, const pos_&  p2){return p1*(!p2);}
inline pos   operator *(const quat_& q1, const pos_&  p2){return pos(q1*p2.p, q1*p2.q);}
inline pos   operator *(const pos_&  p1, const quat_& q2){return pos(p1.p, p1.q*q2);}
inline pos   operator /(const quat_& q1, const pos_&  p2){return q1*(!p2);}
inline pos   operator /(const pos_&  p1, const quat_& q2){return p1*(!q2);}
inline vec3  operator *(const pos_&  p1, const vec3_& x) {return p1.p + p1.q*x;}
inline vec4  operator *(const pos_&  p1, const vec4_& x) {return vec4(x.p*p1.p) + p1.q*x;}

inline pos_&  inverse   (pos_& p)                         {inverse(p.q); p.p = p.q*(-p.p); return p;}
// =======================================
//   matrix_operators & functions
// =======================================
inline double  det       (const mat3_& m)                 {return m.det();}
inline mat3_& negate    (mat3_& m)                       {return m.negate();}
inline mat3_& transpose (mat3_& m)                       {return m.transpose();}
inline mat3_& inverse   (mat3_& m)                       {return m.inverse();}
inline mat3   operator -(const mat3_& m)                 {mat3 ret=m; ret.negate();    return ret;}
inline mat3   operator ~(const mat3_& m)                 {mat3 ret=m; ret.transpose(); return ret;}
inline mat3   operator !(const mat3_& m)                 {mat3 ret=m; ret.inverse();   return ret;}
inline double  det       (const mat4_& m)                 {return m.det();}
inline mat4_& negate    (mat4_& m)                       {return m.negate();}
inline mat4_& transpose (mat4_& m)                       {return m.transpose();}
inline mat4_& inverse   (mat4_& m)                       {return m.inverse();}
inline mat4   operator -(const mat4_& m)                 {mat4 ret=m; ret.negate();    return ret;}
inline mat4   operator ~(const mat4_& m)                 {mat4 ret=m; ret.transpose(); return ret;}
inline mat4   operator !(const mat4_& m)                 {mat4 ret=m; ret.inverse();   return ret;}
// =======================================
//   matrix-double_operators & functions
// =======================================
inline mat3   operator +(double val, const mat3_& m)     {
	return mat3(m.m00+val,m.m01+val,m.m02+val,
				m.m10+val,m.m11+val,m.m12+val,
				m.m20+val,m.m21+val,m.m22+val);
}
inline mat3   operator -(double val, const mat3_& m)     {
	return mat3(m.m00-val,m.m01-val,m.m02-val,
				m.m10-val,m.m11-val,m.m12-val,
				m.m20-val,m.m21-val,m.m22-val);
}
inline mat3   operator *(double val, const mat3_& m)     {
	return mat3(m.m00*val,m.m01*val,m.m02*val,
				m.m10*val,m.m11*val,m.m12*val,
				m.m20*val,m.m21*val,m.m22*val);
}
inline mat3   operator /(double val, const mat3_& m)     {
	return mat3(val/m.m00,val/m.m01,val/m.m02,
				val/m.m10,val/m.m11,val/m.m12,
				val/m.m20,val/m.m21,val/m.m22);
}
inline mat3   operator +(const mat3_& m, double val)     {
	return mat3(m.m00+val,m.m01+val,m.m02+val,
				m.m10+val,m.m11+val,m.m12+val,
				m.m20+val,m.m21+val,m.m22+val);
}
inline mat3   operator -(const mat3_& m, double val)     {
	return mat3(m.m00-val,m.m01-val,m.m02-val,
				m.m10-val,m.m11-val,m.m12-val,
				m.m20-val,m.m21-val,m.m22-val);
}
inline mat3   operator *(const mat3_& m, double val)     {
	return mat3(m.m00*val,m.m01*val,m.m02*val,
				m.m10*val,m.m11*val,m.m12*val,
				m.m20*val,m.m21*val,m.m22*val);
}
inline mat3   operator /(const mat3_& m, double val)     {
	return mat3(m.m00/val,m.m01/val,m.m02/val,
				m.m10/val,m.m11/val,m.m12/val,
				m.m20/val,m.m21/val,m.m22/val);
}
inline mat3_& operator +=(mat3_& m, double val)     {
	m.m00+=val;m.m01+=val;m.m02+=val;
	m.m10+=val;m.m11+=val;m.m12+=val;
	m.m20+=val;m.m21+=val;m.m22+=val;
	return m;
}
inline mat3_& operator -=(mat3_& m, double val)     {
	m.m00-=val;m.m01-=val;m.m02-=val;
	m.m10-=val;m.m11-=val;m.m12-=val;
	m.m20-=val;m.m21-=val;m.m22-=val;
	return m;
}
inline mat3_& operator *=(mat3_& m, double val)     {
	m.m00*=val;m.m01*=val;m.m02*=val;
	m.m10*=val;m.m11*=val;m.m12*=val;
	m.m20*=val;m.m21*=val;m.m22*=val;
	return m;
}
inline mat3_& operator /=(mat3_& m, double val)     {
	m.m00/=val;m.m01/=val;m.m02/=val;
	m.m10/=val;m.m11/=val;m.m12/=val;
	m.m20/=val;m.m21/=val;m.m22/=val;
	return m;
}
inline mat4   operator +(double val, const mat4_& m)     {
	return mat4(m.m00+val,m.m01+val,m.m02+val,m.m03+val,
				m.m10+val,m.m11+val,m.m12+val,m.m13+val,
				m.m20+val,m.m21+val,m.m22+val,m.m23+val,
				m.m30+val,m.m31+val,m.m32+val,m.m33+val);
}
inline mat4   operator -(double val, const mat4_& m)     {
	return mat4(m.m00-val,m.m01-val,m.m02-val,m.m03-val,
				m.m10-val,m.m11-val,m.m12-val,m.m13-val,
				m.m20-val,m.m21-val,m.m22-val,m.m23-val,
				m.m30-val,m.m31-val,m.m32-val,m.m33-val);
}
inline mat4   operator *(double val, const mat4_& m)     {
	return mat4(m.m00*val,m.m01*val,m.m02*val,m.m03*val,
				m.m10*val,m.m11*val,m.m12*val,m.m13*val,
				m.m20*val,m.m21*val,m.m22*val,m.m23*val,
				m.m30*val,m.m31*val,m.m32*val,m.m33*val);
}
inline mat4   operator /(double val, const mat4_& m)     {
	return mat4(val/m.m00,val/m.m01,val/m.m02,val/m.m03,
				val/m.m10,val/m.m11,val/m.m12,val/m.m13,
				val/m.m20,val/m.m21,val/m.m22,val/m.m23,
				val/m.m30,val/m.m31,val/m.m32,val/m.m33);
}
inline mat4   operator +(const mat4_& m, double val)     {
	return mat4(m.m00+val,m.m01+val,m.m02+val,m.m03+val,
				m.m10+val,m.m11+val,m.m12+val,m.m13+val,
				m.m20+val,m.m21+val,m.m22+val,m.m23+val,
				m.m30+val,m.m31+val,m.m32+val,m.m33+val);
}
inline mat4   operator -(const mat4_& m, double val)     {
	return mat4(m.m00-val,m.m01-val,m.m02-val,m.m03-val,
				m.m10-val,m.m11-val,m.m12-val,m.m13-val,
				m.m20-val,m.m21-val,m.m22-val,m.m23-val,
				m.m30-val,m.m31-val,m.m32-val,m.m33-val);
}
inline mat4   operator *(const mat4_& m, double val)     {
	return mat4(m.m00*val,m.m01*val,m.m02*val,m.m03*val,
				m.m10*val,m.m11*val,m.m12*val,m.m13*val,
				m.m20*val,m.m21*val,m.m22*val,m.m23*val,
				m.m30*val,m.m31*val,m.m32*val,m.m33*val);
}
inline mat4   operator /(const mat4_& m, double val)     {
	return mat4(m.m00/val,m.m01/val,m.m02/val,m.m03/val,
				m.m10/val,m.m11/val,m.m12/val,m.m13/val,
				m.m20/val,m.m21/val,m.m22/val,m.m23/val,
				m.m30/val,m.m31/val,m.m32/val,m.m33/val);
}
inline mat4_& operator +=(mat4_& m, double val)     {
	m.m00+=val;m.m01+=val;m.m02+=val;m.m03+=val;
	m.m10+=val;m.m11+=val;m.m12+=val;m.m13+=val;
	m.m20+=val;m.m21+=val;m.m22+=val;m.m23+=val;
	m.m30+=val;m.m31+=val;m.m32+=val;m.m33+=val;
	return m;
}
inline mat4_& operator -=(mat4_& m, double val)     {
	m.m00-=val;m.m01-=val;m.m02-=val;m.m03-=val;
	m.m10-=val;m.m11-=val;m.m12-=val;m.m13-=val;
	m.m20-=val;m.m21-=val;m.m22-=val;m.m23-=val;
	m.m30-=val;m.m31-=val;m.m32-=val;m.m33-=val;
	return m;
}
inline mat4_& operator *=(mat4_& m, double val)     {
	m.m00*=val;m.m01*=val;m.m02*=val;m.m03*=val;
	m.m10*=val;m.m11*=val;m.m12*=val;m.m13*=val;
	m.m20*=val;m.m21*=val;m.m22*=val;m.m23*=val;
	m.m30*=val;m.m31*=val;m.m32*=val;m.m33*=val;
	return m;
}
inline mat4_& operator /=(mat4_& m, double val)     {
	m.m00/=val;m.m01/=val;m.m02/=val;m.m03/=val;
	m.m10/=val;m.m11/=val;m.m12/=val;m.m13/=val;
	m.m20/=val;m.m21/=val;m.m22/=val;m.m23/=val;
	m.m30/=val;m.m31/=val;m.m32/=val;m.m33/=val;
	return m;
}
// =======================================
//   matrix-vector_operators & functions
// =======================================
inline vec4   operator *(const mat4_& m, const vec4_& v)  {
	return vec4(
		m.m00*v.x + m.m01*v.y + m.m02*v.z + m.m03*v.p,
		m.m10*v.x + m.m11*v.y + m.m12*v.z + m.m13*v.p,
		m.m20*v.x + m.m21*v.y + m.m22*v.z + m.m23*v.p,
		m.m30*v.x + m.m31*v.y + m.m32*v.z + m.m33*v.p);
}
inline vec3   operator *(const mat4_& m, const vec3_& v)  {
	return vec3(
		m.m00*v.x + m.m01*v.y + m.m02*v.z + m.m03,
		m.m10*v.x + m.m11*v.y + m.m12*v.z + m.m13,
		m.m20*v.x + m.m21*v.y + m.m22*v.z + m.m23);
}
inline vec4   operator *(const mat3_& m, const vec4_& v)  {
	return vec4(
		m.m00*v.x + m.m01*v.y + m.m02*v.z,
		m.m10*v.x + m.m11*v.y + m.m12*v.z,
		m.m20*v.x + m.m21*v.y + m.m22*v.z,
		v.p);
}
inline vec3   operator *(const mat3_& m, const vec3_& v)  {
	return vec3(
		m.m00*v.x + m.m01*v.y + m.m02*v.z,
		m.m10*v.x + m.m11*v.y + m.m12*v.z,
		m.m20*v.x + m.m21*v.y + m.m22*v.z);
}
inline vec4   operator *(const vec4_& v, const mat4_& m)  {
	return vec4(
		m.m00*v.x + m.m10*v.y + m.m20*v.z + m.m30*v.p,
		m.m01*v.x + m.m11*v.y + m.m21*v.z + m.m31*v.p,
		m.m02*v.x + m.m12*v.y + m.m22*v.z + m.m32*v.p,
		m.m03*v.x + m.m13*v.y + m.m23*v.z + m.m33*v.p);
}
inline vec3   operator *(const vec3_& v, const mat4_& m)  {
	return vec4(
		m.m00*v.x + m.m10*v.y + m.m20*v.z + m.m30,
		m.m01*v.x + m.m11*v.y + m.m21*v.z + m.m31,
		m.m02*v.x + m.m12*v.y + m.m22*v.z + m.m32);
}
inline vec4   operator *(const vec4_& v, const mat3_& m)  {
	return vec4(
		m.m00*v.x + m.m10*v.y + m.m20*v.z,
		m.m01*v.x + m.m11*v.y + m.m21*v.z,
		m.m02*v.x + m.m12*v.y + m.m22*v.z,
		v.p);
}
inline vec3   operator *(const vec3_& v, const mat3_& m)  {
	return vec3(
		m.m00*v.x + m.m10*v.y + m.m20*v.z,
		m.m01*v.x + m.m11*v.y + m.m21*v.z,
		m.m02*v.x + m.m12*v.y + m.m22*v.z);
}
// =======================================
//   matrix-matrix_operators & functions
// =======================================
inline mat3   operator +(const mat3_& m1, const mat3_& m2){
	return mat3(m1.m00+m2.m00,m1.m01+m2.m02,m1.m02+m2.m02,
				m1.m10+m2.m10,m1.m11+m2.m12,m1.m12+m2.m12,
				m1.m20+m2.m20,m1.m21+m2.m22,m1.m22+m2.m22);
}
inline mat3   operator -(const mat3_& m1, const mat3_& m2){
	return mat3(m1.m00-m2.m00,m1.m01-m2.m02,m1.m02-m2.m02,
				m1.m10-m2.m10,m1.m11-m2.m12,m1.m12-m2.m12,
				m1.m20-m2.m20,m1.m21-m2.m22,m1.m22-m2.m22);
}
inline mat3   operator *(const mat3_& m1, const mat3_& m2){
	mat3 ret(0.0f);
	for(int r = 0; r < 3; ++r)
		for(int c = 0; c < 3; ++c)
			for(int i = 0; i < 3; ++i)
				ret.m[r][c] += m1.m[r][i] * m2.m[i][c];
	return ret;
}
inline mat3   operator /(const mat3_& m1, const mat3_& m2){
	return m1*!m2;
}
inline mat3   operator %(const mat3_& m1, const mat3_& m2){
	return !m1*m2;
}
inline mat3_& operator +=(mat3_& m1, const mat3_& m2){
	m1.m00=+m2.m00;m1.m01=+m2.m01;m1.m02=+m2.m02;
	m1.m10=+m2.m10;m1.m11=+m2.m11;m1.m12=+m2.m12;
	m1.m20=+m2.m20;m1.m21=+m2.m21;m1.m22=+m2.m22;
	return m1;
}
inline mat3_& operator -=(mat3_& m1, const mat3_& m2){
	m1.m00=+m2.m00;m1.m01=+m2.m01;m1.m02=+m2.m02;
	m1.m10=+m2.m10;m1.m11=+m2.m11;m1.m12=+m2.m12;
	m1.m20=+m2.m20;m1.m21=+m2.m21;m1.m22=+m2.m22;
	return m1;
}
inline mat3_& operator *=(mat3_& m1, const mat3_& m2){
	m1 = m1*m2;
	return m1;
}
inline mat4   operator +(const mat4_& m1, const mat4_& m2){
	return mat4(m1.m00+m2.m00,m1.m01+m2.m01,m1.m02+m2.m02,m1.m03+m2.m03,
				m1.m10+m2.m10,m1.m11+m2.m11,m1.m12+m2.m12,m1.m13+m2.m13,
				m1.m20+m2.m20,m1.m21+m2.m21,m1.m22+m2.m22,m1.m23+m2.m23,
				m1.m30+m2.m30,m1.m31+m2.m31,m1.m32+m2.m32,m1.m33+m2.m33);
}
inline mat4   operator -(const mat4_& m1, const mat4_& m2){
	return mat4(m1.m00-m2.m00,m1.m01-m2.m01,m1.m02-m2.m02,m1.m03-m2.m03,
				m1.m10-m2.m10,m1.m11-m2.m11,m1.m12-m2.m12,m1.m13-m2.m13,
				m1.m20-m2.m20,m1.m21-m2.m21,m1.m22-m2.m22,m1.m23-m2.m23,
				m1.m30-m2.m30,m1.m31-m2.m31,m1.m32-m2.m32,m1.m33-m2.m33);
}
inline mat4   operator *(const mat4_& m1, const mat4_& m2){
	mat4 ret(0.0f);
	for(int r = 0; r < 4; ++r)
		for(int c = 0; c < 4; ++c)
			for(int i = 0; i < 4; ++i)
				ret.m[r][c] += m1.m[r][i] * m2.m[i][c];
	return ret;
}
inline mat4   operator /(const mat4_& m1, const mat4_& m2){
	return m1*!m2;
}
inline mat4   operator %(const mat4_& m1, const mat4_& m2){
	return !m1*m2;
}
inline mat4_& operator +=(mat4_& m1, const mat4_& m2){
	m1.m00=+m2.m00;m1.m01=+m2.m01;m1.m02=+m2.m02;m1.m03=+m2.m03;
	m1.m10=+m2.m10;m1.m11=+m2.m11;m1.m12=+m2.m12;m1.m13=+m2.m13;
	m1.m20=+m2.m20;m1.m21=+m2.m21;m1.m22=+m2.m22;m1.m23=+m2.m23;
	m1.m30=+m2.m30;m1.m31=+m2.m31;m1.m32=+m2.m32;m1.m33=+m2.m33;
	return m1;
}
inline mat4_& operator -=(mat4_& m1, const mat4_& m2){
	m1.m00=+m2.m00;m1.m01=+m2.m01;m1.m02=+m2.m02;m1.m03=+m2.m03;
	m1.m10=+m2.m10;m1.m11=+m2.m11;m1.m12=+m2.m12;m1.m13=+m2.m13;
	m1.m20=+m2.m20;m1.m21=+m2.m21;m1.m22=+m2.m22;m1.m23=+m2.m23;
	m1.m30=+m2.m30;m1.m31=+m2.m31;m1.m32=+m2.m32;m1.m33=+m2.m33;
	return m1;
}
inline mat4_& operator *=(mat4_& m1, const mat4_& m2){
	m1 = m1*m2;
	return m1;
}


#endif // BASICMATRIX_H
