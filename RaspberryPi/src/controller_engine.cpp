#include "controller_engine.h"
#include "globals.h"
#include <iostream>

ControllerEngine::ControllerEngine(){}

float** ControllerEngine::getTunableVarPtr(){
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
  std::cout <<"controller engine run" << std::endl;
  while(1){
    //Check controller user inputs
    readZmqSub();
    //Alternativly: ControllerEngine::zmqSubReadTraj;

    //If this is not the correct controller
    if ( !(controllerSelect == controllerId) ){
      //Exit while loop (Try next controller)
      std::cout <<"controller select: " << controllerSelect<< " controller ID: " << controllerId <<std::endl;
      break;
    }

    //Wait for spi thread to finish reading sensors and enter sleeping mode
    pthread_mutex_lock(&begin_control_iteration);
    pthread_cond_wait(&start_cond, &begin_control_iteration);
    pthread_mutex_unlock(&begin_control_iteration);

    //Read sensordata while spi thread is sleeping
    readPeriph();

    iterate(controllerObject);

    //Send output to spi
    writeOutput();
  }
}
