#include "controller_engine.h"

JointSpacePosController::JointSpacePosController()
:controllerEngine(){};

ControllerEngine* JointSpacePosController::getHandle(){
  return &controllerEngine;
}
void JointSpacePosController::run(){
  controllerEngine.run();
}

//Customize names that fit your implementation
JointSpacePosController::JointSpacePosController():controllerEngine(){};{
  controllerEngine.iterate = &iterate();
  //ZmqSub inputs
  *name1 = cntrllrEngine.data1;
  *name2 = &cntrllrEngine.data2;
  *name3 = &cntrllrEngine.data3;
  *name4 = &cntrllrEngine.data4;
  *name5 = &cntrllrEngine.data5;
  *name6 = &cntrllrEngine.data6;
  *name7 = &cntrllrEngine.data7;
  *name8 = &cntrllrEngine.data8;
  *name9 = &cntrllrEngine.data9;
  *name10 = &cntrllrEngine.data10;
  *trajSize = &cntrllrEngine.trajSize;
  *trajTimeStamp = &cntrllrEngine.trajTimeStamp;
  *trajPosition = &cntrllrEngine.trajPosition;
  *trajVelocity = &cntrllrEngine.trajVelocity;
  *trajAcceleration = &cntrllrEngine.trajAcceleration;
  //Peripheral inputs
  *jointAngle1 = &cntrllrEngine.jointAngle;
  *jointAngle2 = &cntrllrEngine.jointAngle;
  *angularVel1 = &cntrllrEngine.angularVel1;
  *angularVel2 = &cntrllrEngine.angularVel2;
  *angularAcc1 = &cntrllrEngine.angularAcc1;
  *angularAcc2 = &cntrllrEngine.angularAcc2;
  //Controller output
  *commandedTorque1 = &cntrllrEngine.commandedTorque1;
  *commandedTorque2 = &cntrllrEngine.commandedTorque2;
  //Run time adjustable variables (example: Kp, Ki and so on..)
  *var1 = &cntrllrEngine.var1;
  *var2 = &cntrllrEngine.var2;
  *var3 = &cntrllrEngine.var3;
  *var4 = &cntrllrEngine.var4;
  *var5 = &cntrllrEngine.var5;
  *var6 = &cntrllrEngine.var6;
  *var7 = &cntrllrEngine.var7;
  *var8 = &cntrllrEngine.var8;
  *var9 = &cntrllrEngine.var9;
  *var10 = &cntrllrEngine.var10;
  *var11 = &cntrllrEngine.var11;
  *var12 = &cntrllrEngine.var12;
  *var13 = &cntrllrEngine.var13;
  *var14 = &cntrllrEngine.var14;
  *var15 = &cntrllrEngine.var15;
  *var16 = &cntrllrEngine.var16;
  *var17 = &cntrllrEngine.var17;
  *var18 = &cntrllrEngine.var18;
  *var19 = &cntrllrEngine.var19;
  *var20 = &cntrllrEngine.var20;
}



void JointSpacePosController::iterate(){
  *commandedTorque1 = *name1
  *commandedTorque1 = *name2
}
