#include "controller_engine.h"

class JointSpacePosController {
  public:
    ControllerEngine controllerEngine;

    float *name1;
    float *name2;
    float *name3;
    float *name4;
    float *name5;
    float *name6;
    float *name7;
    float *name8;
    float *name9;
    float *name10;

    int *trajSize;
    float* trajTimeStamp;
    float* trajPosition;
    float* trajVelocity;
    float* trajAcceleration;
    //Peripheral inputs
    float *jointAngle1;
    float *jointAngle2;
    float *angularVel1;
    float *angularVel2;
    float *angularAcc1;
    float *angularAcc2;
    float *commandedTorque1;
    float *commandedTorque2;

    float *var1;
    float *var2;
    float *var3;
    float *var4;
    float *var5;
    float *var6;
    float *var7;
    float *var8;
    float *var9;
    float *var10;
    float *var11;
    float *var12;
    float *var13;
    float *var14;
    float *var15;
    float *var16;
    float *var17;
    float *var18;
    float *var19;
    float *var20;

    JointSpacePosController();
    ControllerEngine* getHandle();
    void iterate();
    static void iterateStatic(void *controller_object);
    void run();
};
