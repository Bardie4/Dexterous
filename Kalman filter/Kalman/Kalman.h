#ifndef Kalman_h
#define Kalman_h
#include <MatrixMath.h>
class Kalman
{
  public:
  //n: state count. m: Output count. d: Measurement count
    int a_size
    int b_size
    int h_size
    int r_size

    //Preallocating space for matrices
    //Max 6 states, 6 measurements and 6 outputs
    mtx_type Q[36];
    mtx_type P_k[36];
    mtx_type S[36];
    mtx_type R[36];
    mtx_type A_[36];
    mtx_type H_[36]
    
    //Preallocating space for calculation
    mtx_type temp1[6];
    mtx_type temp2[6];
    mtx_type temp3[36];
    mtx_type temp4[36];
    mtx_type temp5[6];
    mtx_type temp6[36];
    mtx_type temp7[36];
    mtx_type temp8[36];
    mtx_type temp9[6];
    mtx_type temp10[36];
    mtx_type temp11[36]

    mtx_type x_kk[6];
    mtx_type y_k[6];
    mtx_type K[36];
    mtx_type P_kk[36]
    
  Kalman(float *A, mtx_type *B, mtx_type *H, double *q, double *r, double *p_0 ,int *n, int *m, int *d);
  
  void predict(mtx_type* x_k, mtx_type* u);
  
  void update_(mtx_type* x_k, mtx_type* z);
};
#endif