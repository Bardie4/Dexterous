#include "controllers/ct_pos_controller.h"
#include <math.h>
#include <wiringPi.h>
#include <iostream>
//Customize names that fit your implementation
void CartesianPosController::iterateStatic(void *controller_object){
  return ((CartesianPosController*)controller_object)->iterate();
}

CartesianPosController::CartesianPosController():controllerEngine(){
  controllerEngine.controllerObject = this;
  controllerEngine.iterate = &CartesianPosController::iterateStatic;
  //ZmqSub inputss
  x = &controllerEngine.data1;
  y = &controllerEngine.data2;
  trajSize = &controllerEngine.trajSize;
  trajTimeStamp = controllerEngine.trajTimeStamp;
  trajPosition = controllerEngine.trajPosition;
  trajVelocity = controllerEngine.trajVelocity;
  trajAcceleration = controllerEngine.trajAcceleration;
  //Peripheral inputs
  jointAngle1 = &controllerEngine.jointAngle1;
  jointAngle2 = &controllerEngine.jointAngle2;
  angularVel1 = &controllerEngine.angularVel1;
  angularVel2 = &controllerEngine.angularVel2;
  angularAcc1 = &controllerEngine.angularAcc1;
  angularAcc2 = &controllerEngine.angularAcc2;
  //Controller output
  commandedTorque1 = &controllerEngine.commandedTorque1;
  commandedTorque2 = &controllerEngine.commandedTorque2;
  //Run time adjustable variables (example: Kp, Ki and so on..)
  kp1 = &controllerEngine.var1;
  ki1 = &controllerEngine.var2;
  kd1 = &controllerEngine.var3;
  kp2 = &controllerEngine.var4;
  ki2 = &controllerEngine.var5;
  kd2 = &controllerEngine.var6;

  //Iitial values:
  *kp1 = 0.3 /(2*3.14*3.0/8.0) ;   //0.1 N/m at max possible error
  *kp2 = 0.65 /(2*3.14*3.0/8.0) ;   //0.1 N/m at max possible error
  (*ki1) = 0;
  (*ki2) = 0;
  (*kd1) = 0;
  (*kd2) = 0;

  l1 = 0.0529;
  l2 = 0.0675;
}

void CartesianPosController::iterate(){
  time1=micros();
  step=time1-time0;
  time0=micros();

  //Inverse kinematics. Source: http://www.hessmer.org/uploads/RobotArm/Inverse%2520Kinematics%2520for%2520Robot%2520Arm.pdf
  temp = ( pow((*x), 2) + pow((*y), 2) - pow(l1, 2)-pow(l2, 2)) / (2.0*l1*l2);
  jointAngle2Setpoint = atan2( sqrt( 1.0 - pow(temp, 2) ), temp );
  k1 = l1 + l2 * cos(jointAngle2Setpoint);
  k2 = l2 * sin(jointAngle2Setpoint);
  gamma = atan2(k2, k1);
  jointAngle1Setpoint = atan2( (*y), (*x) ) - gamma;


  if (jointAngle1Setpoint != jointAngle1Setpoint){
    *commandedTorque1 = 0.0;
    *commandedTorque2 = 0.0;
    return;
  }
  if (jointAngle2Setpoint != jointAngle2Setpoint){
    *commandedTorque1 = 0.0;
    *commandedTorque2 = 0.0;
    return;
  }

  //Joint space controller
  error1 = jointAngle1Setpoint - (*jointAngle1);
  integral1 += error1 * (step/1000000.0) * (*ki1);
  if (integral1 > 0.1){
    integral1 = 0.1;
  }else if (integral1 < -0.1){
    integral1 = -0.1;
  }
  *commandedTorque1 = error1 * (*kp1) + integral1 + (*angularVel1) * (*kd1);

  error2 = jointAngle2Setpoint - (*jointAngle2);
  integral2 += error2 * (step/1000000.0) * (*ki2);
  if (integral2 > 0.1){
    integral2 = 0.1;
  }else if (integral2 < -0.1){
    integral2 = -0.1;
  }
  *commandedTorque2 = error2 * (*kp2) + integral2 + (*angularVel2) * (*kd2)- (1.5*(*commandedTorque1)*(18.5/23)*(28.0/18));

  //Print status every 10000 cycles
  itrCounter++;
  if ( itrCounter > 10000){
    std::cout << "Finger "<< controllerEngine.fingerId << " controller: " << controllerEngine.controllerId << " iteration time: " << step << std::endl;
    std::cout << "setpoint1: " << jointAngle1Setpoint << " error1: " << error1 <<" output1: " << *commandedTorque1 << std::endl;
    std::cout << "setpoint2: " << jointAngle2Setpoint << " error2: " << error2 <<" output2: " << *commandedTorque2 << std::endl;
    itrCounter=0;
  }
}

ControllerEngine* CartesianPosController::getHandle(){
  return &controllerEngine;
}

void CartesianPosController::run(){
  std::cout <<"controller bootstrap run" << std::endl;
  controllerEngine.run();
}
