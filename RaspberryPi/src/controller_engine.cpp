#include "controller_engine.h"
#include "globals.h"
#include <iostream>

ControllerEngine::ControllerEngine(){
  trajectoryMessage = 0;
}

float** ControllerEngine::getTunableVarPtr(){
  return varPtrs;
}

void ControllerEngine::readZmqSub(){
  pthread_mutex_lock(&zmqSubLock);
  if (zmqSubMemPtr->newMessage){
    controllerSelect = zmqSubMemPtr->controllerSelect;
    std::cout <<"updated controllerSelect" << std::endl;
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
    if (trajSize > 100){
      trajSize = 100;
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
    if (trajectoryMessage){
      readTrajZmqSub();
      std::cout << "traj" << std::endl;
    }else{
      readZmqSub();
      std::cout <<"simple" <<std::endl;
    }

    if ( !(controllerSelect == controllerId) ){
      //Mark the message as unread
      pthread_mutex_lock(&zmqSubLock);
      zmqSubMemPtr->newMessage = 1;
      pthread_mutex_unlock(&zmqSubLock);

      std::cout <<"controller select: " << controllerSelect<< " controller ID: " << controllerId <<std::endl;
      //Exit while loop (Try next controller)
      break;
    }

    //Wait for spi thread to finish reading sensors and enter sleeping mode
    pthread_mutex_lock(&begin_control_iteration);
    pthread_cond_wait(&start_cond, &begin_control_iteration);
    pthread_mutex_unlock(&begin_control_iteration);

    //Read sensordata
    readPeriph();

    iterate(controllerObject);

    //Send output to spi
    writeOutput();
  }
}
