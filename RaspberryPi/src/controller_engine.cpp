#include "controller_engine.h"
#include <pthread.h>
#include <iostream.h>
extern pthread_mutex_t zmqSubLock = PTHREAD_MUTEX_INITIALIZER;
extern pthread_mutex_t periphLock = PTHREAD_MUTEX_INITIALIZER;
extern pthread_mutex_t begin_control_iteration = PTHREAD_MUTEX_INITIALIZER;
extern pthread_cond_t start_cond = PTHREAD_COND_INITIALIZER;

/*
void ControllerEngine::(*iterate)(void);
*/
float** ControllerEngine::getTunableVarPtr(){

  varPtrs = { &var1, &var2, &var3, &var4, &var5, &var6, &var7, &var8, &var9, &var10,
              &var11, &var12, &var13, &var14, &var15, &var16, &var17, &var18, &var19, &var20};

  return varPtrs;
}

void ControllerEngine::readZmqSub(){
  pthread_mutex_lock(&zmqSubLock);
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
  zmqSubMemPtr->newMessage = 0;
  pthread_mutex_unlock(&zmqSubLock);
}

void ControllerEngine::readTrajZmqSub(){
  pthread_mutex_lock(&zmqSubLock);
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
      trajAcceleration[i] = zmqSubMemPtr->trajAcceleration[i];
    }
    zmqSubMemPtr->newMessage = 0;
  pthread_mutex_unlock(&zmqSubLock);
  }
}
void ControllerEngine::readPeriph(){
  jointAngle1 = periphMemPtr->jointAngle1;
  jointAngle2 = periphMemPtr->jointAngle2;
  angularVel1 = periphMemPtr->angularVel1;
  angularVel2 = periphMemPtr->angularVel2;
}

void ControllerEngine::writeOutput(){
  periphMemPtr->commandedTorque1 = commandedTorque1;
  periphMemPtr->commandedTorque2 = commandedTorque2;
}

void ControllerEngine::run(){
  while(1){
    //Check controller user inputs
    readZmqSub();
    //Alternativly: ControllerEngine::zmqSubReadTraj;

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
    readPeriph();

    iterate();

    //Send output to spi
    writeOutput();

    //OPTIONAL: report status to terminal
    itrCounter++;
    if ( itrCounter > 1000){
      std::cout <<"controller: " << controllerId << "is running on finger: " << fingerId << std::endl;
      itrCounter = 0;
    }

  }
}
