#include "controllers/js_pos_controller.h"
#include <wiringPi.h>
#include <iostream>
//Customize names that fit your implementation
void JointSpacePosController::iterateStatic(void *controller_object){
  return ((JointSpacePosController*)controller_object)->iterate();
}

JointSpacePosController::JointSpacePosController():controllerEngine(){
  controllerEngine.controllerObject = this;
  controllerEngine.iterate = &JointSpacePosController::iterateStatic;

  //********RENAMING VARIABLES FROM ENGINE*******************
  //ZmqSub inputss
  jointAngle1Setpoint = &controllerEngine.data1;
  jointAngle2Setpoint = &controllerEngine.data2;

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
  *kp1 = 0.2 /(2*3.14*3.0/8.0) ;   //0.1 N/m at max possible error
  *kp2 = 0.5 /(2*3.14*3.0/8.0) ;   //0.1 N/m at max possible error
  *ki1 = 0;
  *ki2 = 0;
  *kd1 = 0;
  *kd2 = 0;
}

void JointSpacePosController::iterate(){
  //Step length
  time1=micros();
  step=time1-time0;
  time0=micros();


  //PID controller
  error1 = (*jointAngle1Setpoint) - (*jointAngle1);
  integral1 += error1 * (step/1000000.0) * (*ki1);
  if (integral1 > 0.1){
    integral1 = 0.1;
  }else if (integral1 < -0.1){
    integral1 = -0.1;
  }
  *commandedTorque1 = error1 * (*kp1) + integral1 + (*angularVel1) * (*kd1);

  error2 = *(jointAngle2Setpoint) - (*jointAngle2);
  integral2 += error2 * (step/1000000.0) * (*ki2);
  if (integral2 > 0.1){
    integral2 = 0.1;
  }else if (integral2 < -0.1){
    integral2 = -0.1;
  }
  *commandedTorque2 = error2 * (*kp2) + integral2 + (*angularVel2) * (*kd2);

  //Print status every 10000 cycles
  itrCounter++;
  if ( itrCounter > 10000){
    std::cout << "Finger "<< controllerEngine.fingerId << " controller: " << controllerEngine.controllerId << " iteration time: " << step << std::endl;
    std::cout << "setpoint1: " << ((*jointAngle1Setpoint)*(180.0/3.14)) <<" Angle1: "<< ((*jointAngle1)*(180.0/3.14)) << " error1: " << error1 <<" output1: " << *commandedTorque1 << std::endl;
    std::cout << "setpoint2: " << ((*jointAngle2Setpoint)*(180.0/3.14)) <<" Angle2: "<< ((*jointAngle2)*(180.0/3.14)) << " error1: " << error2 <<" output2: " << *commandedTorque2 << std::endl;
    itrCounter=0;
  }
}

ControllerEngine* JointSpacePosController::getHandle(){
  return &controllerEngine;
}

void JointSpacePosController::run(){
  std::cout <<"controller bootstrap run" << std::endl;
  controllerEngine.run();
}
