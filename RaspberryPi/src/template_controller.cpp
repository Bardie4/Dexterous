#include "controllers/template_controller.h"
extern pthread_mutex_t zmqlock = PTHREAD_MUTEX_INITIALIZER;
extern pthread_mutex_t begin_control_iteration = PTHREAD_MUTEX_INITIALIZER;
extern pthread_mutex_t restart = PTHREAD_MUTEX_INITIALIZER;
extern pthread_cond_t start_cond = PTHREAD_COND_INITIALIZER;
extern extpthread_cond_t restart_cond = PTHREAD_COND_INITIALIZER;

float* TemplateController::bind_to_finger(int finger_id, int controller_id, ZmqFingerMem* zmq_mem_ptr, spiFingerMem* spi_mem_ptr){
  //Let controller know the memory location of controller/sensor inputs/outputs
  zmqMemPtr = zmq_mem_ptr;              //See .zmqRead() & zmqReadTraj()
  spiMemPtr = spi_mem_ptr;              //See .spiRead() & .spiWrite

  //Let controller know which finger it runs on, and its assigned number
  fingerId = finger_id;                 //Usefull when printing to terminal
  controllerId = controller_id;         //See .run()

  //See .getPtrsToVars()
  varPtrs = { &var1, &var2, &var3, &var4, &var5, &var6, &var7, &var8, &var9, &var10,
              &var11, &var12, &var13, &var14, &var15, &var16, &var17, &var18, &var19, &var20};

  //***********EDIT THIS PART*********************
  //Change names of references to fit your implementation
  //data1-4 is controller input from zmq. var1-20 is variables that can be changed at runtime.
  //Unchangeble variables can be added in header file
  float& meaningFullVarName1 = data1;
  float& meaningFullVarName2 = data2;
  float& meaningFullVarName3 = data3;
  float& meaningFullVarName4 = data4;
  float& kp1 = var1;
  float& ki1 = var2;
  float& Kd1 = var3;
  float& kp2 = var4;
  float& ki2 = var5;
  float& Kd2 = var6;

  //Let the finger know the addresses of "tunable" variables in this controller
  return varPtrs;
}

void TemplateController::zmq_read(){
  pthread_mutex_lock(&zmqlock);
  if (zmqMemPtr->newMessage){
    controllerSelect = zmqMemPtr->controllerSelect;
    data1 = zmqMemPtr->data1;
    data2 = zmqMemPtr->data2;
    data3 = zmqMemPtr->data3;
    data4 = zmqMemPtr->data4;
  }
  zmqMemPtr->newMessage = 0;
  pthread_mutex_unlock(&zmqlock);
}

void TemplateController::zmq_read_traj(){
  pthread_mutex_lock(&zmqlock);
  if (zmqMemPtr->newMessage){
    controllerSelect = zmqMemPtr->controllerSelect;
    data1 = zmqMemPtr->data1;
    data2 = zmqMemPtr->data2;
    data3 = zmqMemPtr->data3;
    data4 = zmqMemPtr->data4;
    trajSize = zmqMemPtr->trajSize;
    if (trajSize > 1024){
      trajSize = 1024;
    }
    else if (trajSize < 0){
      trajSize = 0;
    }
    for (int i=0; i < trajSize; i++){
      trajTimeStamp[i] = zmqMemPtr->trajTimeStamp[i];
      trajPosition[i] = zmqMemPtr->trajPosition[i];
      trajVelocity[i] = zmqMemPtr->trajVelocity[i];
      trajAcceleration[i] = zmqMemPtr->Acceleration[i];
    }
    zmqMemPtr->newMessage = 0;
  pthread_mutex_unlock(&zmqlock);
}

void TemplateController::spiRead(){
  jointAngle1 = spiMemPtr->jointAngle1;
  jointAngle2 = spiMemPtr->jointAngle2;
  angularVel1 = spiMemPtr->angularVel1;
  angularVel2 = spiMemPtr->angularVel2;
}

void TemplateController::spi_read(){
  jointAngle1 = spiMemPtr->jointAngle1;
  jointAngle2 = spiMemPtr->jointAngle2;
  angularVel1 = spiMemPtr->angularVel1;
  angularVel2 = spiMemPtr->angularVel2;
}

void TemplateController::spi_write(){
  spiMemPtr->commandedTorque1 = commandedTorque1;
  spiMemPtr->CommandedTorque2 = commandedTorque2;
}

void TemplateController::run(){
  while(1){
    //Check controller user inputs
    TemplateController::zmq_read();
    //Alternativly: TemplateController::zmqReadTraj();

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
    TemplateController::spi_read()



    //***************WRITE CONTROLLER FROM HERE********************


    //Use the now updated sensordata:
    //jointAngle1, jointAngle2 , angularVel1, angularVel2

    //and controller user inputs:
    //data1, data2, data3, data4 / meaningFullVarName(1-4)
    //trajTimeStamp, trajPosition, trajVelocity, trajtrajAcceleration

    //To calculate output:
    //commandedTorque1; commandedTorque2;


    //************* TO HERE****************************************



    //Send output to spi
    TemplateController::spi_write();

    //OPTIONAL: report status to terminal
    itrCounter++;
    if ( itrCounter > 1000){
      std::cout <<"controller: " << controllerId << "is running on finger: " << fingerId << std::endl;
      itrCounter = 0;
    }

  }
}
