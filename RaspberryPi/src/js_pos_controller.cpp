#include "controllers/js_pos_controller.h"
//Customize names that fit your implementation
static void iterateStatic(void *controller_object){
  return ((JointSpacePosController*)controller_object)->iterate();
}

JointSpacePosController::JointSpacePosController():controllerEngine(){
  controllerEngine.controllerObject = this;
  controllerEngine.iterate = &iterateStatic;
  //ZmqSub inputss
  name1 = &controllerEngine.data1;
  name2 = &controllerEngine.data2;
  name3 = &controllerEngine.data3;
  name4 = &controllerEngine.data4;
  name5 = &controllerEngine.data5;
  name6 = &controllerEngine.data6;
  name7 = &controllerEngine.data7;
  name8 = &controllerEngine.data8;
  name9 = &controllerEngine.data9;
  name10 = &controllerEngine.data10;
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
  var1 = &controllerEngine.var1;
  var2 = &controllerEngine.var2;
  var3 = &controllerEngine.var3;
  var4 = &controllerEngine.var4;
  var5 = &controllerEngine.var5;
  var6 = &controllerEngine.var6;
  var7 = &controllerEngine.var7;
  var8 = &controllerEngine.var8;
  var9 = &controllerEngine.var9;
  var10 = &controllerEngine.var10;
  var11 = &controllerEngine.var11;
  var12 = &controllerEngine.var12;
  var13 = &controllerEngine.var13;
  var14 = &controllerEngine.var14;
  var15 = &controllerEngine.var15;
  var16 = &controllerEngine.var16;
  var17 = &controllerEngine.var17;
  var18 = &controllerEngine.var18;
  var19 = &controllerEngine.var19;
  var20 = &controllerEngine.var20;
}

void JointSpacePosController::iterate(){
  *commandedTorque1 = *name1;
  *commandedTorque1 = *name2;
}

ControllerEngine* JointSpacePosController::getHandle(){
  return &controllerEngine;
}

void JointSpacePosController::run(){
  controllerEngine.run();
}
