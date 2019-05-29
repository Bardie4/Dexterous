
#include "globals.h"
#include <math.h>
//#include <zmq.h>
//#include "zmq/zhelpers.h"
#include <zmq.hpp>
#include <stdio.h>
#include <pigpio.h>
#include <unistd.h>
#include <algorithm>
//#include <bitset>
#include "pthread.h"
#include <iostream>
#include <string>
#include <wiringPi.h>
#include <sstream>
#include "generated_flattbuffers/simple_instructions_generated.h"
#include "generated_flattbuffers/finger_broadcast_generated.h"
#include "generated_flattbuffers/trajectory_msg_generated.h"
#include "controllers/js_pos_controller.h"
#include "controllers/ct_pos_controller.h"


/*
static pthread_mutex_t zmqSubLock = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t periphLock = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t begin_control_iteration = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t start_cond = PTHREAD_COND_INITIALIZER;
*/
pthread_t tid[10];
pthread_t i2c_threads[7];
//micro seconds between sensor reads.
//(the process of reading adds additional time to the total step length)
#define ITR_DEADLINE 1000

//Flatbuffers
using namespace my_schemas; // Specified in the schema.
flatbuffers::FlatBufferBuilder pubBuilder(1024);
uint8_t *buffer_pointer;

void free_me_from_my_suffering(void *data, void *hint){
    free (data);
}

class Finger{

  public:

		//count controller cycles
		short id;
    short controllerSelect;

		//Variables used for calibration)
    int spiHandle;
		unsigned csAngleSensor1;
		unsigned csAngleSensor2;
    int i2cHandle;
    float theta1Zero;
    float theta2Zero;

    //Pointer to list of parameters for twenty different controllers
    //float* controllerParameters[20];


    //Shared memory
    ZmqSubFingerMem zmqSubSharedMem;
    PeripheralFingerMem periphSharedMem;

    //Controllers
    JointSpacePosController jsPosCntrllr;
    CartesianPosController ctPosCntrllr;

    void bindController(ControllerEngine* handle, short controller_id){
        handle->zmqSubMemPtr = &zmqSubSharedMem;
        handle->periphMemPtr = &periphSharedMem;
        handle->fingerId = id;
        handle->controllerId = controller_id;
        //controllerParameters[controller_id] = handle->varPtrs;
    }

    void adjustControllerParameter(){
      pthread_mutex_lock(&zmqSubLock);
      controllerSelect = zmqSubSharedMem.controllerSelect;
      float controller = zmqSubSharedMem.data1;
      float param =  (int) zmqSubSharedMem.data2;
      float newParamValue = zmqSubSharedMem.data3;
      pthread_mutex_unlock(&zmqSubLock);

      if ( !(controllerSelect == 999) ){
        //Exit while loop (Try next controller)
        return;
      }
      if ( controller < 0 || controller > 20 ){
        //Select one of twenty controllers or break
        return;
      }

      if ( param < 0 || param > 20 ){
        //Select one of twenty parameters or break
        return;
      }

    }

    //Constructor
    Finger(int identity)
      :jsPosCntrllr(), ctPosCntrllr(){
      id= identity;
      std::cout << "binding controllers" <<std::endl;
      bindController(&jsPosCntrllr.controllerEngine, 2);
      bindController(&ctPosCntrllr.controllerEngine, 3);
      zmqSubSharedMem.runFlag=0;
      periphSharedMem.runFlag=0;
      periphSharedMem.commandedTorque1 = 0;
      periphSharedMem.commandedTorque2 = 0;

    }

		void shutdown(){
			//Tell zmq client that the thread is no longer active
      std::cout << "shutting down thread: " << id <<std::endl;
			pthread_mutex_lock(&periphLock);
      periphSharedMem.runFlag = 0;
			pthread_mutex_unlock(&periphLock);
			//Tell zmq function to no longer measure sesnors for this finger
			pthread_mutex_lock(&zmqSubLock);
      zmqSubSharedMem.runFlag = 0;
			pthread_mutex_unlock(&zmqSubLock);
			}

		void calibration(){

			std::cout << "Hold on, im calibrating finger " << id << std::endl;
			char read_angle_cmd[]= {0b00000000,0b00000000};
			char torque_cmd[3];
			int gpioResult;
			int spiResult;
      char inBuf[2];
      uint16_t theta1Zero16;
      uint16_t theta2Zero16;

      //*********FIND START POINT OF SENSOR 1**********
			//***********************************************
			torque_cmd[0]=(uint8_t) 0b00000010;
			torque_cmd[1]=(uint8_t) 90; //Skift retning
			torque_cmd[2]=(uint8_t) 120;  //link2
			pthread_mutex_lock(&periphLock);
      i2cWriteDevice(i2cHandle, torque_cmd, 3);
			pthread_mutex_unlock(&periphLock);
			printf("Driving to endpoint for link 1\n");
			sleep(3);

			//READ ANGLE AT END POINT
			pthread_mutex_lock(&periphLock);
			gpioResult = gpioWrite(csAngleSensor1,0);
			spiResult = spiXfer(spiHandle, read_angle_cmd, inBuf, 2);
			gpioResult = gpioWrite(csAngleSensor1,1);
			pthread_mutex_unlock(&periphLock);
			theta1Zero16 = inBuf[0] << 8;
      theta1Zero16 = theta1Zero16 +  inBuf[1];
      theta1Zero = (theta1Zero16 * 2.0*3.14159) / 65535.0;


	    //*********FIND START POINT OF SENSOR 2**********
      //***********************************************
      torque_cmd[0]=(uint8_t) 0b00000001;
      torque_cmd[1]=(uint8_t) 90; //Skift retning
      torque_cmd[2]=(uint8_t) 120;  //link2
      pthread_mutex_lock(&periphLock);
      i2cWriteDevice(i2cHandle, torque_cmd, 3);
      pthread_mutex_unlock(&periphLock);
      printf("Driving to endpoint for link 2\n");
      sleep(4);

      //READ ANGLE AT END POINT
			pthread_mutex_lock(&periphLock);
			gpioResult = gpioWrite(csAngleSensor2,0);
			spiResult = spiXfer(spiHandle, read_angle_cmd, inBuf, 2);
			gpioResult = gpioWrite(csAngleSensor2,1);
		 	pthread_mutex_unlock(&periphLock);
      theta2Zero16 = inBuf[0] << 8;
      theta2Zero16 = theta2Zero16 +  inBuf[1];
      theta2Zero = (theta2Zero16 * 2.0*3.14159) / 65535.0;


      //*****************STOP MOTORS*******************
			//***********************************************
			torque_cmd[0]=(uint8_t) 0;
			torque_cmd[1]=(uint8_t) 0;
			torque_cmd[2]=(uint8_t) 0;
      pthread_mutex_lock(&periphLock);
      i2cWriteDevice(i2cHandle, torque_cmd, 3);
			pthread_mutex_unlock(&periphLock);

      std::cout << "Zero point 1: " << theta1Zero <<" zero point 2: " << theta2Zero << std::endl;

			//Tell SPI thread to include sensors in measurement loop
			pthread_mutex_lock(&periphLock);
			periphSharedMem.runFlag = 1;
			pthread_mutex_unlock(&periphLock);
			//Wait for a controller to be selected
			while(1){
				sleep(2);
        pthread_mutex_lock(&zmqSubLock);
        controllerSelect = zmqSubSharedMem.controllerSelect;
        pthread_mutex_unlock(&zmqSubLock);
				if ( !(controllerSelect==1) ){
            std::cout <<"Exiting now because controller select is: " << controllerSelect << std::endl;
					break;
        }
      std::cout << "Finger: "<< id <<" is waiting for controller to be selected. Current selection: " << controllerSelect <<std::endl;
      }
		}

    void* run(){
			std::cout << "i am thread: "<< id << "  >:O" << std::endl;
      pthread_mutex_lock(&zmqSubLock);
      controllerSelect = zmqSubSharedMem.controllerSelect;
      pthread_mutex_unlock(&zmqSubLock);
			calibration();
			//While finger is instructed to be active
      while( !(controllerSelect == 0) ){
          jsPosCntrllr.run();
          ctPosCntrllr.run();
          adjustControllerParameter();
        //  std::cout <<"id after bind: "<<jsPosCntrllr.controllerEngine.controllerId<<std::endl;
          pthread_mutex_lock(&zmqSubLock);
          controllerSelect = zmqSubSharedMem.controllerSelect;
          pthread_mutex_unlock(&zmqSubLock);
          usleep(500);
      }
			shutdown();
    }

};

class ZmqSubscriber{

  private:
  ZmqSubFingerMem* fingerMemPtr[7];
  ZmqSubFingerMem fingerMem;
  Finger* fingerPtrs[7];

  bool oldRunFlag;

  //ZMQ
  zmq::context_t context;
  zmq::socket_t subscriber;
  char* address[5];
  public:

    ZmqSubscriber()
      :context(1) , subscriber(context, ZMQ_SUB){
      //ZMQ setup: http://zguide.zeromq.org/cpp:wuclient
      const char *filter = "B";
      subscriber.setsockopt(ZMQ_SUBSCRIBE, filter, strlen (filter));
      //subscriber.connect("tcp://10.218.131.31:5563");
      subscriber.connect("tcp://169.254.27.157:5563");


      /*
      context = zmq_ctx_new ();
      subscriber = zmq_socket (context, ZMQ_SUB);
      zmq_connect (subscriber, "tcp://169.254.27.157:5563");
      zmq_setsockopt (subscriber, ZMQ_SUBSCRIBE, "B", 1);*/
      for (int i = 0; i<7; i++){
        fingerMemPtr[i] = NULL;
      }

      fingerMem.runFlag = 1;
      fingerMem.newMessage = 1;
    }

    void bindFinger(Finger* finger){
      if ( (finger->id) > 6 || (finger->id) < 0){
        std::cout << "Binding finger " << (finger->id) <<" to zmq subscriber"
        << "failed: Invalid identity. Choose a unique identity between 0 and 6"
        << std::endl;
        return;
      }
      if ( fingerMemPtr[finger->id] != NULL){
        std::cout << "Warning: Finger ID "<< finger->id << " is already in use."
        << "Choose a unique identity between 0 and 6." << std::endl;
        return;
      }

      fingerMemPtr[finger->id] = &(finger->zmqSubSharedMem);
      fingerPtrs[finger->id] = finger;
    }

    static void *initFinger(void *finger_object){
      std::cout << "New thread created for finger: "<< ((Finger*)finger_object)->id << std::endl;
      return ((Finger*)finger_object)->run();
    }

    void passOnSimpleInstructions(zmq::message_t* buffer){
      //std::cout <<"entering pass function. message is of size: "<< buffer->size() <<std::endl;
      //Parse flattbuffer and store it
      auto messageObj = GetSimpleInstructionMsg(buffer->data());
      //  std::cout <<"created flattbuffer object" <<std::endl;
      //  std::cout <<"Finger selected: " << messageObj->finger_select() <<std::endl;
      fingerMem.fingerSelect = messageObj->finger_select();
      //Return if selected finger is not valid
      if ( (fingerMem.fingerSelect < 0) || (fingerMem.fingerSelect > 6) ){
        return;
      }
      if (fingerMemPtr[fingerMem.fingerSelect] == NULL){
        return;
      }
      fingerMem.controllerSelect = messageObj->controller_select();;
      fingerMem.data1 = messageObj->data1();
      fingerMem.data2 = messageObj->data2();
      fingerMem.data3 = messageObj->data3();
      fingerMem.data4 = messageObj->data4();
      fingerMem.data5 = messageObj->data5();
      fingerMem.data6 = messageObj->data6();
      fingerMem.data7 = messageObj->data7();
      fingerMem.data8 = messageObj->data8();
      fingerMem.data9 = messageObj->data9();
      fingerMem.data10 = messageObj->data10();

      //std::cout << fingerMem.fingerSelect << " " << fingerMem.controllerSelect << std::endl;
      pthread_mutex_lock(&zmqSubLock);
      //Read runflag before its overwritten
      oldRunFlag = fingerMemPtr[fingerMem.fingerSelect]->runFlag;
      //Overwrite previous instructions
      *fingerMemPtr[fingerMem.fingerSelect] = fingerMem;

      //If finger is not already running
      if ( (oldRunFlag == 0) && !(fingerMem.controllerSelect == 0)){
        //Start the finger on a new thread.
        std::cout << "Attempting to create thread"<< (2+fingerMem.fingerSelect) << std::endl;
        pthread_create(&(tid[2+fingerMem.fingerSelect]), NULL, &initFinger, fingerPtrs[fingerMem.fingerSelect]);
      }
      pthread_mutex_unlock(&zmqSubLock);
    }

    void passOnTrajectoryMsg(zmq::message_t* buffer){
      //std::cout <<"entering pass function. message is of size: "<< buffer->size() <<std::endl;
      //Parse flattbuffer and store it
      auto messageObj = GetTrajectoryMsg(buffer->data());
      //  std::cout <<"created flattbuffer object" <<std::endl;
      //  std::cout <<"Finger selected: " << messageObj->finger_select() <<std::endl;
      fingerMem.fingerSelect = messageObj->finger_select();
      //Return if selected finger is not valid
      if ( (fingerMem.fingerSelect < 0) || (fingerMem.fingerSelect > 6) ){
        return;
      }
      if (fingerMemPtr[fingerMem.fingerSelect] == NULL){
        return;
      }
      fingerMem.controllerSelect = messageObj->controller_select();;
      fingerMem.data1 = messageObj->data1();
      fingerMem.data2 = messageObj->data2();
      fingerMem.data3 = messageObj->data3();
      fingerMem.data4 = messageObj->data4();
      fingerMem.data5 = messageObj->data5();
      fingerMem.data6 = messageObj->data6();
      fingerMem.data7 = messageObj->data7();
      fingerMem.data8 = messageObj->data8();
      fingerMem.data9 = messageObj->data9();
      fingerMem.data10 = messageObj->data10();
      fingerMem.trajSize = messageObj->trajSize();
      auto timeStamp = messageObj->trajTimeStamp();
      auto position = messageObj->trajPosition();
      auto velocity = messageObj->trajVelocity();
      auto acceleration = messageObj->trajAcceleration();
      for (int i= 0; i < fingerMem.trajSize ; i++){
        fingerMem.trajTimeStamp[i] = timeStamp->Get(i);
        fingerMem.trajPosition[i] = position->Get(i);
        fingerMem.trajVelocity[i] = velocity->Get(i);
        fingerMem.trajAcceleration[i] = acceleration->Get(i);
      }

      //std::cout << fingerMem.fingerSelect << " " << fingerMem.controllerSelect << std::endl;
      pthread_mutex_lock(&zmqSubLock);
      //Read runflag before its overwritten
      oldRunFlag = fingerMemPtr[fingerMem.fingerSelect]->runFlag;
      //Overwrite previous instructions
      *fingerMemPtr[fingerMem.fingerSelect] = fingerMem;

      //If finger is not already running
      if ( (oldRunFlag == 0) && !(fingerMem.controllerSelect == 0)){
        //Start the finger on a new thread.
        std::cout << "Attempting to create thread"<< (2+fingerMem.fingerSelect) << std::endl;
        pthread_create(&(tid[2+fingerMem.fingerSelect]), NULL, &initFinger, fingerPtrs[fingerMem.fingerSelect]);
      }
      pthread_mutex_unlock(&zmqSubLock);
    }

    void* run(){
      while(1){
        //Listen for messages
        //From guide: http://zguide.zeromq.org/cpp:interrupt
        zmq::message_t buffer;
        zmq::message_t address;
        subscriber.recv(&address);
        subscriber.recv(&buffer);

      //  std::cout <<"Message type: " <<flatbuffers::GetBufferIdentifier(buffer.data()) << std::endl;

        if ( SimpleInstructionMsgBufferHasIdentifier( buffer.data() ) ){
          passOnSimpleInstructions(&buffer);
        }else if (TrajectoryMsgBufferHasIdentifier( buffer.data() )) {
          passOnTrajectoryMsg(&buffer);
        }
      }
    }

		static void* start(void* zmq_sub_object){
			return ((ZmqSubscriber*)zmq_sub_object)->run();
		}
};

class PeripheralsController{
  private:
    //Pointer to memory of finger, and local copies.
		PeripheralFingerMem* fingerMemPtr[7];
    PeripheralFingerMem fingerMem[7];
    PeripheralFingerMem fingerMemPrev[7];
    float* zeroAngle[7][2];
    bool zeroCross[7][2];
    unsigned (*csAndI2cAddr)[3];

    unsigned spiFrequency;
    unsigned spiChannel;
    int spiHandle;
    int gpioResult;
    int spiResult;

    int i2cHandle;
    unsigned i2cReg;

    char inBuf[4];
    char outBuf[4];
    char read_command_8;
    char read_command_16[2];
    uint16_t angle16;
    uint8_t angle8;
    float angleRad;
    float Output1Scaled8;       //Per
    float Output2Scaled8;   //Scaled for 8bit, but not actually 8bit.
    float maxTorqLink1;         //The maximum torque capability link1
    float maxTorqLink2;         //The maximum torque capability link1
		int time0;
		int time1;
		int step;
    int itr_counter;
    /*
    zmq::context_t context;
    zmq::socket_t publisher;
    */
    void* context;
    void* publisher;

    float torque1;
    float torque2;
    int i2cHandles[7];

    std::vector<flatbuffers::Offset<FingerStates>> handStates;
    int size;
    uint8_t *buf;

    //boost::asio::deadline_timer timer;

    float readAngle8(unsigned &cs){
      outBuf[0] = read_command_8;
			pthread_mutex_lock(&periphLock);
      gpioResult = gpioWrite(cs,0);
      spiResult = spiXfer(spiHandle, outBuf, inBuf, 1);
      gpioResult = gpioWrite(cs,1);
      angle8 = inBuf[0];
      pthread_mutex_unlock(&periphLock);
      angleRad = (angle8 * 2.0*3.14159) / 255.0;
      return angleRad;
    }

    float readAngle12(unsigned &cs){
      outBuf[0] = read_command_16[0];
      outBuf[1] = read_command_16[1];
			pthread_mutex_lock(&periphLock);
      gpioResult = gpioWrite(cs,0);
      spiResult = spiXfer(spiHandle, outBuf, inBuf, 2);
      gpioResult = gpioWrite(cs,1);
      angle16 = inBuf[0] << 8;
      angle16 = angle16 + inBuf[1];
      //std::cout <<"Raw 16 bit angle: "<< angle16 << " on chip select: "<< cs <<" inbuf:"<<unsigned(inBuf[0])<<unsigned(inBuf[1])<<std::endl;
			pthread_mutex_unlock(&periphLock);
      angleRad = (angle16 * 2.0*3.14159) / 65535.0;
      return angleRad;
    }

		void writeOutput8(int &i2c_handle, float &output1, float &output2){
      //Sends 3 bytes. first byte carries flags for direction
      //2nd. byte carries absolute value of torque for link1
      //3rd. byte carries absolute value of torque for link2

      Output1Scaled8 =( (output1 / maxTorqLink1) * 255.0);
      Output2Scaled8 =( (output2 / maxTorqLink2) * 255.0);

      //Set flags and make torque positive
      outBuf[0] = 0;
			if ( Output1Scaled8 < 0 ){
				outBuf[0] = 0b00000001;
        Output1Scaled8 = Output1Scaled8*(-1.0);
			}
      if ( Output2Scaled8 < 0 ){
        outBuf[0] += 0b00000010;
        Output2Scaled8 = Output2Scaled8*(-1.0);
      }
      //Cap the values at maximum torque capability of each link
			if (Output1Scaled8 > 255.0){
				Output1Scaled8 = 255.0;
			}
			if (Output2Scaled8 > 255.0){
				Output2Scaled8 = 255.0;
			}
      //Cast to unsigned 8 bit, and put into output buffer
			outBuf[1] =  (uint8_t) Output1Scaled8;
			outBuf[2] =  (uint8_t) Output2Scaled8;

      //Send
      /*
      if ( ( i2cHandle = i2cOpen(1, i2cAddress, 0) ) < 0 ){
        std::cout << "i2cOpen() failed for adress: " << i2cAddress << std::endl;
      }*/
      i2cWriteDevice(i2c_handle, outBuf, 3);
      //i2cReadDevice(i2c_handle, outBuf, 3);
      /*
      if ( i2cClose( i2cHandle ) < 0 ){
        std::cout << "i2cClose failed! Handle: " << i2cHandle << std::endl;
      }*/

      torque1 = outBuf[1] * maxTorqLink1;
      torque2 = outBuf[2] * maxTorqLink2;
      if (outBuf[0] & 0b000000001){
        torque1 = -1.0*torque1;
      }
      if (outBuf[0] & 0b00000010){
        torque2 = -1.0*torque2;
      }

		}

  public:
    PeripheralsController(unsigned cs_and_i2c_addr[7][3]){
      //   :context (1), publisher (context, ZMQ_PUB){

      csAndI2cAddr = cs_and_i2c_addr;

      //***********SPI AND GPIO INITIALIZATION***************
      //The spiChannel is assosiated with a chip select pin. This pin must
      //not be connected to anything, the GPIO pins specified in csAndI2cAddr
      //is used instead.
    	spiChannel = 0;
      spiFrequency = 15000000;
      if (gpioInitialise() < 0){
        std::cout << "Pigpio initialisation failed. Run program as superuser"
        << std::endl;
      }
      if ( (spiHandle = spiOpen(spiChannel, spiFrequency, 0) ) < 0 ){
        std::cout << "spiOpen() failed" << std::endl;
      } else {
        std::cout << "SPI is open. Hande: " << spiHandle << std::endl;
      }

      for (int i=0; i<7; i++){
        i2cHandles[i] = i2cOpen(1, csAndI2cAddr[i][2], 0);
      }

      //Data is not transmitted on SPI when cs is high.
      //Therefore; set all to high on startup
      for (int i=0; i <= 6; i++){
        for (int j=0; j <2; j++){
          gpioResult = gpioSetMode(csAndI2cAddr[i][j], PI_OUTPUT);
          gpioResult = gpioWrite(csAndI2cAddr[i][j], 1);
        }
      }

      //SPI commands used by  MagAlpha MA302  sensors.
      read_command_8 = 0b00000000;
      read_command_16[0] = 0b00000000;
      read_command_16[1] = 0b00000000;

      //Maximum torque output allowed.
      //Must be set to same value in ESP32 and Raspberry, or it will also
      //act as a gain
      maxTorqLink1= 0.1;         //The maximum torque capability link1
      maxTorqLink2= 0.1;         //The maximum torque capability link2

      //
      for (int i=0; i<7; i++){
        fingerMemPtr[i] = NULL;
      }

      //ZMQ publisher
      /*
      publisher.bind("tcp://*:5564");
      */
      handStates.reserve(7);
      context = zmq_ctx_new ();
      publisher = zmq_socket (context, ZMQ_PUB);
      zmq_bind (publisher, "tcp://*:5564");

      for (int i= 0; i <7; i++){
        for (int j=0; j <2; j++){
          zeroCross[i][j] = 0;
          zeroAngle[i][j] = 0;
        }
      }
      time0=micros();
      for (int i = 0; i<7; i++){
        fingerMemPrev[i].jointAngle1 = 0;
        fingerMemPrev[i].jointAngle2 = 0;
        fingerMemPrev[i].angularVel1 = 0;
        fingerMemPrev[i].angularVel2 = 0;
      }

    }

    void bindFinger (Finger* finger){
      //Check if finger identity is valid. 7 fingers are supported. They need
      //To have a unique identity.
      if ( (finger->id) > 6 || (finger->id) < 0){
        std::cout << "Binding finger " << (finger->id) <<" to pheripherals"
        << "failed: Choose an identity between 0 and 6." << std::endl;
        return;
      }
      if ( fingerMemPtr[finger->id] != NULL){
        std::cout << "Warning: Finger ID "<< finger->id << " is already in use."
        << "Choose a unique identity between 0 and 6." << std::endl;
        return;
      }

      //The finger threads and periphersal thread use this memory to communicate
      fingerMemPtr[finger->id] = &(finger->periphSharedMem);
      zeroAngle[finger->id][0] = &finger->theta1Zero;
      zeroAngle[finger->id][1] = &finger->theta2Zero;
      //During calibration, the finger takes control over the pheripherals.
      //Here it it given the means to do so.
      finger->spiHandle = spiHandle;
      finger->csAngleSensor1 = csAndI2cAddr[finger->id][0];
      finger->csAngleSensor2 = csAndI2cAddr[finger->id][1];
      finger->i2cHandle = i2cHandles[finger->id];
    }

    void* run(){
			while(1){
        //timer.expires_from_now(boost::posix_time::milliseconds(1));
				//Load run flag of fingers that are bound
        //Also load commanded torque if runflag is active
				pthread_mutex_lock(&periphLock);
				for (int i=0; i<7; i++){
            if (fingerMemPtr[i] != NULL){
						        fingerMem[i].runFlag = fingerMemPtr[i]->runFlag;
                    if (fingerMem[i].runFlag == 1){
                      fingerMem[i].commandedTorque1 = fingerMemPtr[i]->commandedTorque1;
                      fingerMem[i].commandedTorque2 = fingerMemPtr[i]->commandedTorque2;
                    }
            }
				}
				pthread_mutex_unlock(&periphLock);

        //Calculate steplength
        time1=micros();
        step=time1-time0;
        time0=micros();
        handStates.clear();

				for (int i=0; i<7; i++){
					//If finger is active
					if (fingerMem[i].runFlag){

						//Read sensors (store it locally)
            fingerMem[i].jointAngle1 = readAngle12(csAndI2cAddr[i][0]);   //Read angle raw
            if (fingerMem[i].jointAngle1 < *zeroAngle[i][0] - 0.3){              //Check if it has crossed zero point
              zeroCross[i][0] = 1;
            }else{
              zeroCross[i][0] = 0;
            }
            //std::cout << "raw angle" << fingerMem[i].jointAngle1 << "zeroAgnle:"<<  zeroAngle[i][0] << "zero cross "<<zeroCross[i][0] <<std::endl;
            fingerMem[i].jointAngle1 = (90.0*3.142/180.0) - (fingerMem[i].jointAngle1 - *zeroAngle[i][0] + (6.283*zeroCross[i][0]));


            //std::cout << "adjusted angle" << fingerMem[i].jointAngle1 << std::endl;
            fingerMem[i].jointAngle2 = readAngle12(csAndI2cAddr[i][1]);   //Read angle raw
            if (fingerMem[i].jointAngle2 <  *zeroAngle[i][1] - 0.3){              //Check if it has crossed zero point
              zeroCross[i][1] = 1;
            }else{
              zeroCross[i][1] = 0;
            }
            fingerMem[i].jointAngle2 = (135.0*3.142/180.0) - (fingerMem[i].jointAngle2 - *zeroAngle[i][1] + (6.283 * zeroCross[i][1])) ;
            if (zeroCross[i][1]){
              std::cout <<"ZERO CROSS" << std::endl;
            }

						//Process sensor information (store it locally)

          //  std::cout <<"new value" << fingerMem[i].jointAngle1 <<" old value: "<< fingerMemPrev[i].jointAngle1<<std::endl;
            fingerMem[i].angularVel1 = (fingerMem[i].jointAngle1 - fingerMemPrev[i].jointAngle1)/(step/1000000.0);
            fingerMem[i].angularVel2 = (fingerMem[i].jointAngle2 - fingerMemPrev[i].jointAngle2)/(step/1000000.0);

            fingerMem[i].angularVel1 = (fingerMem[i].angularVel1 + fingerMemPrev[i].angularVel1)/2.0;
            fingerMem[i].angularVel2 = (fingerMem[i].angularVel2 + fingerMemPrev[i].angularVel2)/2.0;
            //std::cout <<"steplength" << (step/1000000.0) <<" Delta angle: "<< (fingerMem[i].jointAngle1 - fingerMemPrev[i].jointAngle1)<<std::endl;
            fingerMemPrev[i].jointAngle1 = fingerMem[i].jointAngle1;
            fingerMemPrev[i].jointAngle2 = fingerMem[i].jointAngle2;
            fingerMemPrev[i].angularVel1 = fingerMem[i].angularVel1;
            fingerMemPrev[i].angularVel2 = fingerMem[i].angularVel2;

						pthread_mutex_lock(&periphLock);
						//Update memory of finger
						fingerMemPtr[i]->jointAngle1 = fingerMem[i].jointAngle1;
						fingerMemPtr[i]->jointAngle2 = fingerMem[i].jointAngle2;
						fingerMemPtr[i]->angularVel1 = fingerMem[i].angularVel1;
						fingerMemPtr[i]->angularVel2 = fingerMem[i].angularVel2;
						pthread_mutex_unlock(&periphLock);

						//Send output to motor
						writeOutput8(i2cHandles[i], fingerMem[i].commandedTorque1, fingerMem[i].commandedTorque2);

            //Load into flatbuffer struct
            auto fingerStates= CreateFingerStates(pubBuilder,  i,  fingerMem[i].jointAngle1,       fingerMem[i].jointAngle2,
                                                                fingerMem[i].angularVel1,       fingerMem[i].angularVel2,
                                                                0,                              0,
                                                                fingerMem[i].commandedTorque1,  fingerMem[i].commandedTorque2,
                                                                0,0,0,0);
            handStates.push_back(fingerStates);
					}
				}

        //Diagnostics
        itr_counter++;
        if (itr_counter >10000){
          std::cout << "Peripherals thread used: "<< step <<" microseconds on one iteration. (Including "<< (int)ITR_DEADLINE << " us delay). Max elements: "<<handStates.capacity() << " elements: "<< handStates.size() <<std::endl;
          itr_counter=0;
        }

        //timer.wait();
        //Allow fingers to do another iteration on the active controller
        pthread_mutex_lock(&begin_control_iteration);
        pthread_cond_broadcast(&start_cond);
        pthread_mutex_unlock(&begin_control_iteration);
        //Give controllers time to finish an iteration

        //Finish flatbuffer
        auto hand = pubBuilder.CreateVector(handStates);
        auto handBroadcast = CreateHandBroadcast(pubBuilder, hand);
        FinishHandBroadcastBuffer(pubBuilder, handBroadcast);
        //Send
        buf = pubBuilder.GetBufferPointer();
        size = pubBuilder.GetSize();
        /*
        zmq::message_t zmqPubMsg(buf, size);
        publisher.send(zmqPubMsg);
        */
        if (handStates.size()>0){
          zmq_send (publisher, buf, size, 0);
        }

        pubBuilder.Clear();

        usleep(ITR_DEADLINE);
			}
    }

		static void* start(void *periph_controller_object){
			return ((PeripheralsController* )periph_controller_object)->run();
		}

    ~PeripheralsController(){
      if (spiClose(spiHandle) < 0){
        printf("spi destroctor\n");
        printf("Bad handle");
      }
      gpioTerminate();
      printf("we destructed");
    }
};

main(){
  //Initiate finger objects. The arguments is the identity of the finger.
  //The identity corresponds to specific SPI pins. Choose a value between 0-6.
  //Additional fingers can be added (max 7 with the amount of GPIO pins on a RaspberryPi).

	unsigned csSens1_f1 = 16;
	unsigned csSens2_f1 = 23;
	unsigned i2cEsp32_f1 = 0x28;

  unsigned csSens1_f2 = 21;
	unsigned csSens2_f2 = 21;
	unsigned i2cEsp32_f2 = 0x29;

  unsigned csSens1_f3 = 21;
	unsigned csSens2_f3 = 21;
	unsigned i2cEsp32_f3 = 0x29;

  unsigned csSens1_f4 = 21;
	unsigned csSens2_f4 = 21;
	unsigned i2cEsp32_f4 = 0x29;

  unsigned csSens1_f5 = 21;
	unsigned csSens2_f5 = 21;
	unsigned i2cEsp32_f5 = 0x29;

  unsigned csSens1_f6 = 21;
	unsigned csSens2_f6 = 21;
	unsigned i2cEsp32_f6 = 0x29;

  unsigned csSens1_f7 = 21;
	unsigned csSens2_f7 = 21;
	unsigned i2cEsp32_f7 = 0x29;

  //Chip select pins and i2c addresses is packed into array which is to be
  //passed to the "pheripherals" object.
	unsigned csAndI2cAddr[7][3] = 	{{csSens1_f1, csSens2_f1, i2cEsp32_f1}
						 	 			             ,{csSens1_f2, csSens2_f2, i2cEsp32_f2}
      							 			      ,{csSens1_f3, csSens2_f3, i2cEsp32_f3}
        									 			,{csSens1_f4, csSens2_f4, i2cEsp32_f4}
        									 			,{csSens1_f5, csSens2_f5, i2cEsp32_f5}
        									 			,{csSens1_f6, csSens2_f6, i2cEsp32_f6}
        									 			,{csSens1_f7, csSens2_f7, i2cEsp32_f7}};


  PeripheralsController periphContrl( csAndI2cAddr );
  ZmqSubscriber zmqSub;
/*
  Finger finger[7] = {0, 1, 2, 3, 4, 5, 6};
  for (int = i; i<7; i++){
    finger[i].setID(i);
    periphContrl.bindFinger( &finger[i] );
    zmqSub.bindFinger( &finger[i] ) ;
  }
*/
  Finger finger0(0);
  Finger finger1(1);
  Finger finger2(2);
	Finger finger3(3);
	Finger finger4(4);
	Finger finger5(5);
	Finger finger6(6);

  periphContrl.bindFinger( &finger0);
  periphContrl.bindFinger( &finger1);
  periphContrl.bindFinger( &finger2);
  periphContrl.bindFinger( &finger3);
  periphContrl.bindFinger( &finger4);
  periphContrl.bindFinger( &finger5);
  periphContrl.bindFinger( &finger6);

  zmqSub.bindFinger( &finger0);
  zmqSub.bindFinger( &finger1);
  zmqSub.bindFinger( &finger2);
  zmqSub.bindFinger( &finger3);
  zmqSub.bindFinger( &finger4);
  zmqSub.bindFinger( &finger5);
  zmqSub.bindFinger( &finger6);

  pthread_create(&(tid[0]), NULL, &PeripheralsController::start, &periphContrl);
  pthread_create(&(tid[1]), NULL, &ZmqSubscriber::start, &zmqSub);

  pthread_join(tid[0], NULL);
  pthread_join(tid[1], NULL);
}
