#ifndef CONTROLLER_ENGINE
#define CONTROLLER_ENGINE
#include "controller_structs.h"
extern pthread_mutex_t zmqSubLock;
extern pthread_mutex_t periphLock;
extern pthread_mutex_t begin_control_iteration;
extern pthread_cond_t start_cond;

class ControllerEngine {
  //This is a template of a controller, se also cpp file in src folder.
  private:
  //REQUIRED: Local copies of variables shared by zmq thread
  short controllerSelect;
  float data1;
  float data2;
  float data3;
  float data4;
  float data5;
  float data6;
  float data7;
  float data8;
  float data9;
  float data10;
  int trajSize;
  float trajTimeStamp[1024];
  float trajPosition[1024];
  float trajVelocity[1024];
  float trajAcceleration[1024];
  //REQUIRED: Local copies of variables shared by spi thread
  float jointAngle1;
  float jointAngle2;
  float angularVel1;
  float angularVel2;
  float commandedTorque1;
  float commandedTorque2;

  //***********EDIT THIS PART**************
  //When bindToFinger() is called, these references is assigned to data1-4,
  //Give them meaningfull names to help readability of your controller code
  /*
  float& meaningFullVarName1;
  float& meaningFullVarName2;
  float& meaningFullVarName3;
  float& meaningFullVarName4;
*/
  //OPTIONAL: Used to count iterations
  int itrCounter;

  public:
  short fingerId;
  short controllerId;
  PeripheralFingerMem* periphMemPtr;
  ZmqSubFingerMem* zmqSubMemPtr;

  //20 variables that are tunable via ZMQ.
  float var1, var2, var3, var4, var5, var6, var7, var8, var9, var10;
  float var11, var12, var13, var14, var15, var16, var17, var18, var19, var20;
  float* varPtrs[20] = { &var1, &var2, &var3, &var4, &var5, &var6, &var7, &var8, &var9, &var10,
                &var11, &var12, &var13, &var14, &var15, &var16, &var17, &var18, &var19, &var20};
  //*************EDIT THIS PART************
  //When bindToFinger() is called, these references is assigned to var1-20.
  //Give the refrences names that are meaningful to your implementation.
  //Example names variables needed for two pid controllers:
  /*
  float& kp1;
  float& ki1;
  float& Kd1;
  float& kp2;
  float& ki2;
  float& Kd2;
  */

  //This is analogous to a constructor.
  //Lets the controller know which finger it runs on, and assigns it a unique number
  //The controller gets access to memory updated by the spi thread and zmq thread
  //Returns addresses of tunable variables to finger
  float** getTunableVarPtr();
  void readZmqSub();           //reads user inputs from zmq: controllerSelect & data1-4
  void readTrajZmqSub();       //Same as above, but also reads trajectory data
  void readPeriph();           //reads sensor data: joint angle and joint velocity
  void writeOutput();          //Writes commanded torque to memory shared with spi thread.
  void run();               //The actual control loop.
  void (*iterate)(void);
};
#endif
