#ifndef __MYLIB_MATH_MATHBASE__
#define __MYLIB_MATH_MATHBASE__
#include <math.h>


const float PIf = 3.1415927f;
const float PIx2f = 6.2831853f;
const float R2Df = 57.2957802f;
const float D2Rf = 0.0174533f;



inline void  swap(float& a, float& b){float temp = a; a = b; b = temp;}
inline float sign(float a){return (a<0 ? -1.0f : 1.0f);}
inline float abs(float a){return (a>=0 ? a : -a);}
inline float sqrtp(float a){return (a>=0 ? sqrt(a) : 0);}

inline float max(float a, float b){return (a>=b ? a : b);}
inline float min(float a, float b){return (a>=b ? b : a);}



inline float sqrtp(double a){return (a>=0 ? sqrt(a) : 0);}

//#ifndef PI
//#define	PI				 3.1415927f
//#endif
//#ifndef PIx2
//#define	PIx2			 6.2831853f
//#endif
//#ifndef R2D
//#define R2D				57.2957802f
//#endif
//#ifndef D2R
//#define D2R				 0.0174533f
//#endif





#endif

