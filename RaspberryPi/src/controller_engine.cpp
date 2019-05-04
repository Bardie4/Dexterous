#include "controllers/template_controller.h"
#include "finger.h"
extern pthread_mutex_t zmqSubLock = PTHREAD_MUTEX_INITIALIZER;
extern pthread_mutex_t periphLock = PTHREAD_MUTEX_INITIALIZER;
extern pthread_mutex_t begin_control_iteration = PTHREAD_MUTEX_INITIALIZER;
extern pthread_cond_t start_cond = PTHREAD_COND_INITIALIZER;

float* TemplateController::bind_to_finger(short controller_id, Finger* finger){
  //Let controller know the memory location of controller/sensor inputs/outputs
  zmqSubMemPtr = finger->zmqSubSharedMem;              //See .zmqRead() & zmqReadTraj()
  periphMemPtr = finger->periphSharedMem;              //See .spiRead() & .spiWrite

  //Let controller know which finger it runs on, and its assigned number
  fingerId = finger->id;                 //Usefull when printing to terminal
  controllerId = controller_id;         //See .run()

  //See .getPtrsToVars()
  varPtrs = { &var1, &var2, &var3, &var4, &var5, &var6, &var7, &var8, &var9, &var10,
              &var11, &var12, &var13, &var14, &var15, &var16, &var17, &var18, &var19, &var20};

  //***********EDIT THIS PART*********************
  //Change names of references to fit your implementation
  //data1-4 is controller input from zmq. var1-20 is variables that can be changed at runtime.
  //Unchangeble variables can be added in header file
  /*
  float& meaningFullVarName1 = data1;
  float& meaningFullVarName2 = data2;
  float& meaningFullVarName3 = data3;
  float& meaningFullVarName4 = data4;
  float& meaningFullVarName5 = data5;
  float& meaningFullVarName6 = data6;
  float& meaningFullVarName7 = data7;
  float& meaningFullVarName8 = data8;
  float& meaningFullVarName9 = data9;
  float& meaningFullVarName10 = data10;

  float& kp1 = var1;
  float& ki1 = var2;
  float& Kd1 = var3;
  float& kp2 = var4;
  float& ki2 = var5;
  float& Kd2 = var6;
  */

  //Let the finger know the addresses of "tunable" variables in this controller
  return varPtrs;
}

void TemplateController::readZmqSub(){
  pthread_mutex_lock(&zmqlock);
  if (zmqSubMemPtr->newMessage){
    controllerSelect = zmqSubMemPtr->controllerSelect;
    data1 = zmqSubMemPtr->data1;
    data2 = zmqSubMemPtr->data2;
    data3 = zmqSubMemPtr->data3;
    data4 = zmqSubMemPtr->data4;
    data5 = zmqSubMemPtr->data5;
    data6 = zmqSubMemPtr->data6;
    data7 = zmqSubMemPtr->data7;
    data8 = zmqSubMemPtr->data8;
    data9 = zmqSubMemPtr->data9;
    data10 = zmqSubMemPtr->data10;
  }
  zmqMemPtr->newMessage = 0;
  pthread_mutex_unlock(&zmqlock);
}

void TemplateController::readTrajZmqSub(){
  pthread_mutex_lock(&zmqlock);
  if (zmqSubMemPtr->newMessage){
    controllerSelect = zmqSubMemPtr->controllerSelect;
    data1 = zmqSubMemPtr->data1;
    data2 = zmqSubMemPtr->data2;
    data3 = zmqSubMemPtr->data3;
    data4 = zmqSubMemPtr->data4;
    data5 = zmqSubMemPtr->data5;
    data6 = zmqSubMemPtr->data6;
    data7 = zmqSubMemPtr->data7;
    data8 = zmqSubMemPtr->data8;
    data9 = zmqSubMemPtr->data9;
    data10 = zmqSubMemPtr->data10;
    trajSize = zmqSubMemPtr->trajSize;
    if (trajSize > 1024){
      trajSize = 1024;
    }
    else if (trajSize < 0){
      trajSize = 0;
    }
    for (int i=0; i < trajSize; i++){
      trajTimeStamp[i] = zmqSubMemPtr->trajTimeStamp[i];
      trajPosition[i] = zmqSubMemPtr->trajPosition[i];
      trajVelocity[i] = zmqSubMemPtr->trajVelocity[i];
      trajAcceleration[i] = zmqSubMemPtr->Acceleration[i];
    }
    zmqMemPtr->newMessage = 0;
  pthread_mutex_unlock(&zmqlock);
}

void TemplateController::readPeriph(){
  jointAngle1 = periphMemPtr->jointAngle1;
  jointAngle2 = periphMemPtr->jointAngle2;
  angularVel1 = periphMemPtr->angularVel1;
  angularVel2 = periphMemPtr->angularVel2;
}

void TemplateController::writeOutput(){
  periphMemPtr->commandedTorque1 = commandedTorque1;
  periphMemPtr->CommandedTorque2 = commandedTorque2;
}

void TemplateController::run(){
  while(1){
    //Check controller user inputs
    TemplateController::readZmqSub();
    //Alternativly: TemplateController::zmqSubReadTraj;

    //If this is not the correct controller
    if ( !(controllerSelect == controllerId) ){
      //Exit while loop (Try next controller)
      break;
    }

    //Wait for spi thread to finish reading sensors and enter sleeping mode
    pthread_mutex_lock(&begin_control_iteration);
    pthread_cond_wait(&start_cond, &begin_control_iteration);
    pthread_mutex_unlock(&begin_control_iteration);

    //Read sensordata while spi thread is sleeping
    TemplateController::readPeriph()

    iterate();

    //Send output to spi
    TemplateController::writeOutput();

    //OPTIONAL: report status to terminal
    itrCounter++;
    if ( itrCounter > 1000){
      std::cout <<"controller: " << controllerId << "is running on finger: " << fingerId << std::endl;
      itrCounter = 0;
    }

  }
}
