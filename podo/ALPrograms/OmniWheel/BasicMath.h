#ifndef BASICMATH_H
#define BASICMATH_H

#include <math.h>
#include <stdio.h>

const double PIf = 3.1415927f;
const double PIx2f = 6.2831853f;
const double R2Df = 57.2957802f;
const double D2Rf = 0.0174533f;

inline void  swap(double& a, double& b){double temp = a; a = b; b = temp;}
inline double sign(double a){return (a<0 ? -1.0f : 1.0f);}
inline double abs(double a){return (a>=0 ? a : -a);}
inline double sqrtp(double a){return (a>=0 ? sqrt(a) : 0);}

inline double max(double a, double b){return (a>=b ? a : b);}
inline double min(double a, double b){return (a>=b ? b : a);}


#endif // BASICMATH_H
