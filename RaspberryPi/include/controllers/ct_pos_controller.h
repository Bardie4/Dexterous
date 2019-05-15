#include "controller_engine.h"

class CartesianPosController {
  public:
    ControllerEngine controllerEngine;

    float *x;
    float *y;

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

    float *kp1;
    float *ki1;
    float *kd1;
    float *kp2;
    float *ki2;
    float *kd2;

    //Printing
    int itrCounter;
    int time1;
    int step;
    int time;

    //Controller specific variables
    float jointAngle2Setpoint;
    float jointAngle1Setpoint;
    float l1;
    float l2;
    float temp;
    float k1;
    float k2;
    float gamma;
    float error1;
    float error2;
    float integral1;
    float integral2;

    static void iterateStatic(void *controller_object);
    CartesianPosController();
    ControllerEngine* getHandle();
    void iterate();
    void run();
};
