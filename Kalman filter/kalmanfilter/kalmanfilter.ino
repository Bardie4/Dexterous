#include <Kalman.h>
#include <MatrixMath.h>

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

  // System parameters
  double m1=0.03388; //Plastic weight: 0.02388
  double m2=0.02444;
  double l1=5.8;
  double lc1=2.9;
  double lc2=2.9;
  double I1=m1*lc1*lc1;
  double I2=m2*lc2*lc2;
  double kt=0.049;
  double rr1=25.0;
  double rr2=25.0;
  double g=9.81;

  //Kalman initialization
  double h=0.001;
  double p_0[4]={0.0 , 0.0 , 0.0 , 0.0};
  double q[4]={0.9 , 0.9 , 0.9 , 0.9};
  double r[2]={0.99, 0.99}; //two measurements
  mtx_type H[8];
  memset(H,0,8);
  H[0]=1;
  H[7]=1;
  mtx_type A[16];
  memset(A,0,16);
  A[0]=1;
  A[5]=1;
  A[10]=1;
  A[15]=1;
  A[1]=h;
  A[11]=h;
  mtx_type B[8];
  memset(B,0,8);
  B[2]=h;
  B[7]=h;
  Kalman kfilter(A, B, H, q, r, p_0, 4, 2, 2);

  //System states
  double theta1, theta2, theta1_dot, theta2_dot;
  mtx_type theta[2], theta_dot[2], x_m[4], x_k[4];
  memset(x_k,0,4);

  //Statespace matrices
  mtx_type M[4], C[4], G[2];
  
  //Feedforward
  mtx_type C_theta_dot[2];   //Coriolis feedforward
  mtx_type C_theta_dot_g[2]; //Coriolis + gravity
  double tau_c1,tau_c2;
  mtx_type tau_c[2];         //Cogging torque feed forward                    
  mtx_type u_ff[2];          //Total Feedforward (Coriolis + Gravity + Cogging)

  //Outputs
  mtx_type a_ref[2];         //Commanded acceleration (from Controller)
  mtx_type u[2];             //Feedback linearized commanded acceleration
  mtx_type i_ref[2];         //Commanded current (To motor driver)

  //Calculation variables
  double pre1,pre2;

  while(1){
    //***********LOADING AND PREPARING SENSORDATA AND LOOKUPTABLE VALUES**********
    //measure
    kfilter.update_(x_k, x_m);
  
    //Putting data into matrices
    theta[1]=x_k[0];
    theta[2]=x_k[2];
    theta1=x_k[0];
    theta2=x_k[2];
    theta1_dot=x_k[1];
    theta2_dot=x_k[3];
    theta_dot[1]=x_k[1];
    theta_dot[2]=x_k[3];
    tau_c[1]=tau_c1;
    tau_c[2]=tau_c2;
    
    //***********FEEDBACK LINEARIZATION*****************************************
    //Based on equation 8.21 Robot modeling and control, with added cogging torque tau_c,
    //Units in amps instead of torque (tau=i*kt*rr)
    
    //pre calc(To avoid recalculations)
    pre1=-m2*l1*lc2*sin(theta1);
    pre2=m2*lc2*g*cos(theta1+theta2);
  
    //Inertia matrix: Eq. 7.83 Robot modeling and control
    M[1]=(m1*lc1*lc1+m2*(l1*l1+lc2*lc2+2.0*l1*lc2*cos(theta2))+I1+I2)/(kt*rr1);
    M[2]=(m2*(lc2*lc2+l1*lc2*cos(theta2))+I2)/(kt*rr1);
    M[3]=M[2];
    M[4]=(m2*lc2*lc2+I2)/(kt*rr2);
  
    //Coriolis matrix: Eq. 7.88 Robot modeling and control
    C[1]=(pre1*theta2_dot)/(kt*rr1);
    C[2]=(pre1*(theta1_dot+theta2_dot))/(kt*rr1);
    C[3]=(-pre1*theta1_dot)/(kt*rr2);
    C[4]=0;
  
   //Gravuty matrix: Eq. 7.85 Robot modeling and control
    G[1]=((m1*lc1+m2*l1)*g*cos(theta1)+pre2)/(kt*rr1);
    G[2]=pre2/(kt*rr2);
  
    //*************OUTPUT^********************************************************
  
    //Feedforward:
    Matrix.Multiply((mtx_type*)C, (mtx_type*)theta_dot, 2, 2, 1, (mtx_type*)C_theta_dot); //Feed forward: coriolis
    Matrix.Add((mtx_type*)C_theta_dot, (mtx_type*)G, 2, 1, (mtx_type*)C_theta_dot_g);     //Feed forward: coriolis + gravity
    Matrix.Add((mtx_type*)C_theta_dot_g, (mtx_type*)tau_c, 2, 1, (mtx_type*)u_ff);        //Feed forward: coriolis + gravity + cogging torque
  
    //Feedback control signal
    Matrix.Multiply((mtx_type*)M, (mtx_type*)a_ref, 2, 2, 1, (mtx_type*)u);               //Feedback linearized command signal
  
    //Total output signal (In wanted amps)
    Matrix.Add((mtx_type*)u_ff,(mtx_type*)u, 2, 1, (mtx_type*)i_ref);                     //Total output control signal (Feedback+feedforward) in amps.
    kfilter.predict(x_k, a_ref);
  };

};
