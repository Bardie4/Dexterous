#include "Kalman.h"
Kalman::Kalman(mtx_type *A_inn, mtx_type *B_inn, mtx_type *H_inn, double *q, double *r, double *p_0 ,int n_inn, int m_inn, int d_inn)
{
  A=A_inn;
  B=B_inn;
  H=H_inn;
  
  n=n_inn;
  m=m_inn;
  d=d_inn; 
 
  a_size=n*n;
  b_size=n*m;
  h_size=d*n;
  r_size=d*d;

  //Constructing matrices
  memset(Q,0,a_size);
  memset(R,0,r_size);
  memset(P_k,0,a_size);
  for (int i=0; i<n; i++){
   Q[i*(n+1)]=q[i];
   P_k[i*(n+1)]=p_0[i];
  }
  for (int i=0; i<d; i++){
       R[i*(d+1)]=r[i];
  }
  Matrix.Transpose(A, n,n ,(mtx_type*) A_);
  Matrix.Transpose(H, d,n ,(mtx_type*) H_);
}
void Kalman::predict(mtx_type* x_k, mtx_type* u){
  Matrix.Multiply(A, x_k, n, n, 1, (mtx_type*)temp1);
  Matrix.Multiply(B, u, n, m, 1, (mtx_type*)temp2);
  Matrix.Add((mtx_type*)temp1, (mtx_type*)temp2, n, 1, x_kk);
  memcpy(x_kk,x_k,n);

  Matrix.Multiply(A, (mtx_type*)P_k, n, n, n, (mtx_type*)temp3);
  Matrix.Multiply((mtx_type*)temp3, (mtx_type*)A_, n, n, n, (mtx_type*)temp4);
  Matrix.Add((mtx_type*)temp4, (mtx_type*)Q, n, n, (mtx_type*)P_k);
}

void Kalman::update_(mtx_type* x_k, mtx_type* z){
  Matrix.Multiply(H, (mtx_type*)x_kk, d, n, 1, (mtx_type*)temp5);
  Matrix.Subtract((mtx_type*)y_k, (mtx_type*)temp5, d, 1, (mtx_type*)y_k);

  Matrix.Multiply(H, (mtx_type*)P_k, d, n, n, (mtx_type*)temp6);
  Matrix.Multiply((mtx_type*)temp6, (mtx_type*)H_, d, n, d, (mtx_type*)temp7);
  Matrix.Add((mtx_type*)temp7, (mtx_type*)R, d, d, (mtx_type*)S);

  Matrix.Multiply((mtx_type*)P_k, (mtx_type*)H_,n,n,d, (mtx_type*)temp8);
  Matrix.Multiply((mtx_type*)temp8, (mtx_type*)S, n, d, d, (mtx_type*)K);

  Matrix.Multiply((mtx_type*)K, (mtx_type*)y_k, n, d, 1, (mtx_type*) temp9);
  Matrix.Add((mtx_type*)x_k, (mtx_type*)temp9, n, 1, (mtx_type*) x_kk);
  memcpy(x_kk,x_k,n);

  Matrix.Multiply((mtx_type*)K, H, n, d, n, (mtx_type*) temp10);
  Matrix.Multiply((mtx_type*) temp10, (mtx_type*)P_k, n, n, n, (mtx_type*) temp11);
  Matrix.Subtract((mtx_type*)P_k, (mtx_type*)temp11, n, n, (mtx_type*)P_kk);
  memcpy(P_kk,P_k,a_size);
}
