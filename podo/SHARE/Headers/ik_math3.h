#ifndef IK_MATH3_H
#define IK_MATH3_H

#define PI			3.141592653589793

#define MAX(a,b) a>b ? a:b
#define MIN(a,b) a<b ? a:b

double *vector(unsigned int n);
double **matrix(unsigned int row, unsigned int col);
void free_vector(double *vector);
void free_matrix(double **matrix, unsigned int row, unsigned int col);

int mult_mv(const double **matrix_mxn, unsigned int m, unsigned int n, const double vector_nx1[], double result_mx1[]);	// matrix x vector
int mult_mv(const double matrix_mxn[], unsigned int m, unsigned int n, const double vector_nx1[], double result_mx1[]);	// matrix x vector
int mult_mm(const double **matrix_m1xn1, unsigned int m1, unsigned int n1, const double **matrix_n1xn2, unsigned int n2, double **result_m1xn2); // matrix x matrix
int mult_mm(const double matrix_m1xn1[], unsigned int m1, unsigned int n1, const double matrix_n1xn2[], unsigned int n2, double result_m1xn2[]); // matrix x matrix
int mult_mm34(const double matrix_m1xn1[], unsigned int m1, unsigned int n1, const double matrix_n1xn2[], unsigned int n2, double result_m1xn2[]); // matrix x matrix
double dot(const double vector1_nx1[], unsigned int n, const double vector2_nx1[]); // vector dot vector

int sum_mm(const double **matrix1_mxn, unsigned int m, unsigned int n, const double **matrix2_mxn, double **result_mxn); // matrix + matrix
int sum_mm(const double matrix1_mxn[], unsigned int m, unsigned int n, const double matrix2_mxn[], double result_mxn[]); // matrix + matrix
int sum_smsm(double scalar1, const double **matrix1_mxn, unsigned int m, unsigned int n, double scalar2, const double **matrix2_mxn, double **result_mxn); // scalar1*matrix + scalar2*matrix
int sum_smsm(double scalar1, const double matrix1_mxn[], unsigned int m, unsigned int n, double scalar2, const double matrix2_mxn[], double result_mxn[]); // scalar1*matrix + scalar2*matrix
int diff_mm(const double **matrix1_mxn, unsigned int m, unsigned int n, const double **matrix2_mxn, double **result_mxn); // matrix - matrix
int diff_mm(const double matrix1_mxn[], unsigned int m, unsigned int n, const double matrix2_mxn[], double result_mxn[]); // matrix - matrix
int sum_vv(const double vector1_mx1[], unsigned int m, const double vector2_mx1[], double result_mx1[]); // vector + vector
int diff_vv(const double vector1_mx1[], unsigned int m, const double vector2_mx1[], double result_mx1[]); // vector - vector
int mult_sv(const double vector_nx1[], unsigned int n, double scalar, double result_nx1[]);		// scalar * vector
int mult_sm(const double **matrix_mxn, unsigned int m, unsigned int n, double scalar, double **result_mxn); // scalar * matrix
int mult_sm(const double matrix_mxn[], unsigned int m, unsigned int n, double scalar, double result_mxn[]); // scalar * matrix
int subs_v(const double vector_nx1[], unsigned int n, double result_nx1[]);		// result_nx1 = vector_nx1
int subs_sv(double scalar, const double vector_nx1[], unsigned int n, double result_nx1[]);		// result_nx1 = scalar*vector_nx1
int subs_m(const double **matrix_mxn, unsigned int m, unsigned int n, double **result_mxn); // result_mxn = matrix_mxn
int subs_m(const double matrix_mxn[], unsigned int m, unsigned int n, double result_mxn[]); // result_mxn = matrix_mxn

int sum_svsv(double scalar1, const double vector1_nx1[], unsigned int n, double scalar2, const double vector2_nx1[], double result_nx1[]); // s1*v1 + s2*v2
int mult_smv(double scalar, const double **matrix_mxn, unsigned int m, unsigned int n, const double vector_nx1[], double result_mx1[]);	// scalar * matrix * vector
int mult_smv(double scalar, const double matrix_mxn[], unsigned int m, unsigned int n, const double vector_nx1[], double result_mx1[]);	// scalar * matrix * vector
int mult_smm(double scalar, const double **matrix_m1xn1, unsigned int m1, unsigned int n1, const double **matrix_n1xn2, unsigned int n2, double **result_m1xn2); // scalar*matrix*matrix
int mult_smm(double scalar, const double matrix_m1xn1[], unsigned int m1, unsigned int n1, const double matrix_n1xn2[], unsigned int n2, double result_m1xn2[]); // scalar*matrix*matrix
int transpose(double scalar, const double **matrix_mxn, unsigned int m, unsigned int n, double **result_nxm); // scalar * matrix transpose
int transpose(double scalar, const double matrix_mxn[], unsigned int m, unsigned int n, double result_nxm[]); // scalar * matrix transpose
int transpose(double scalar, double **matrix_mxn, unsigned int m, unsigned int n); // scalar * matrix transpose
int transpose(double scalar, double matrix_mxn[], unsigned int m, unsigned int n); // scalar * matrix transpose
int transpose34(double scalar, double matrix_mxn[], unsigned int m, unsigned int n); // scalar * matrix transpose
int cross(double scalar, const double vector1_3x1[], const double vector2_3x1[], double result_3x1[]);	// scalar * vector cross product
int mult_vvt(double scalar, const double vector_mx1[], unsigned int m, const double vector_nx1[], unsigned int n, double **result_mxn); // vector * vector transpose
int mult_vvt(double scalar, const double vector_mx1[], unsigned int m, const double vector_nx1[], unsigned int n, double result_mxn[]); // vector * vector transpose
int mult_vtm(double scalar, const double vector_mx1[], unsigned int m, const double **matrix_mxn, unsigned int n, double result_nx1[]); // scalar * vector transpose * matrix
int mult_vtm(double scalar, const double vector_mx1[], unsigned int m, const double matrix_mxn[], unsigned int n, double result_nx1[]); // scalar * vector transpose * matrix

double norm_v(const double vector_nx1[], unsigned int n);
double dnorm_v(const double vector_nx1[], unsigned int n);

int findminmax(double data[], unsigned int n, double &result_max, double &result_min, unsigned int &i_max, unsigned int &i_min);
int findmax(double data[], unsigned int n, double &result_max, unsigned int &i_max);
int findmin(double data[], unsigned int n, double &result_min, unsigned int &i_min);

int roundoff(double x);
double bound(double input, double bu, double bl);

int inv(double **Ai, int n);
int inv(double Ai[], int n);
int pinv_SR(const double **A, int m, int n, double lambda, double **Ai);
int pinv_SR(const double A[], int m, int n, double lambda, double Ai[]);

int QT2DC(const double qt_4x1[], double **DC_3x3);		// convert a quaternion to a direction cosine matrix
int QT2DC(const double qt_4x1[], double DC_3x3[]);		// convert a quaternion to a direction cosine matrix
int DC2QT(const double **DC_3x3, double qt_4x1[]);		// convert a direction cosine matrix to a quaternion
int DC2QT(const double DC_3x3[], double qt_4x1[]);		// convert a direction cosine matrix to a quaternion
int QTcross(const double qt1_4x1[], const double qt2_4x1[], double result_4x1[]); // qt1 x qt2
int QTtransform(const double qt_4x1[], const double v_3x1[], double result_3x1[]); // vector transformation
int QT2RV(const double qt_4x1[], double rv_4x1[]); // quaternion to rotational vector
int RV2QT(const double rv_4x1[], double qt_4x1[]); // rotational vector to quaternion

int RX(double theta, double **R_3x3); // rotation matrix
int RX(double theta, double R_3x3[]); // rotation matrix
int RY(double theta, double **R_3x3); // rotation matrix
int RY(double theta, double R_3x3[]); // rotation matrix
int RZ(double theta, double **R_3x3); // rotation matrix
int RZ(double theta, double R_3x3[]); // rotation matrix
int qtRX(double theta, double qt_4x1[]); // rotation quaternion
int qtRY(double theta, double qt_4x1[]); // rotation quaternion
int qtRZ(double theta, double qt_4x1[]); // rotation quaternion

double one_cos2(double time, double mag, double time_init, double time_end); // 1-cos() interpolation, (time:current time, mag:magnitude, time_init~time_end:interpolation period)
int one_cos_orientation2(double time, const double qt0_4x1[], const double qt1_4x1[], double time_init, double time_end, double result_4x1[]); // 1-cos() interpolation for orientation from qt0 to qt1, (time:current time, mag:magnitude, time_init~time_end:interpolation period)
int one_cos_orientation3(double time, const double qt0_4x1[], const double qt1_4x1[], double time_init, double time_end, char long_direction, double result_4x1[]); // (long_direction==1) decides the interpolation direction
double trapezoid2(double t, double mag, double T0, double T1, double Tacc_ratio);
double trapezoid_orientation2(double t, const double qt0_4x1[], const double qt1_4x1[], double T0, double T1, double Tacc_ratio, double result_4x1[]);
double trapezoid_orientation3(double t, const double qt0_4x1[], const double qt1_4x1[], double T0, double T1, double Tacc_ratio, char long_direction, double result_4x1[]);

int diff_QT(const double des_qt_4x1[], const double qt_4x1[], double result_3x1[]);

#endif /* IK_MATH3_H */
