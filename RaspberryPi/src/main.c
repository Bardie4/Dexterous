// gcc -Wall -pthread -o bbSPIx_test bbSPIx_test.c -lpigpio
// sudo ./bbSPIx_test

//  Hello World client


#include "zhelpers.h"
#include <stdio.h>
#include <pigpio.h>
#include <unistd.h>
//#include <bitset>
#include "pthread.h"

#define FREQ (15000000)
#define SCLK (11)
#define MOSI (10)
#define MISO 9
#define CS0 8

typedef struct spi_setup{
	int freq;
	int clk;
	int mosi;
	int miso;
	int cs_angle_sensor_1;     //GPIO pins for manual chip select
	int cs_angle_sensor_2;     //GPIO pins for manual chip select
	int cs_esp32;              //GPIO pins for manual chip select
}spi_setup;

const struct spi_setup SPI_F1 = {FREQ ,SCLK, MOSI, MISO, 2, 3, 4};
const struct spi_setup SPI_F2 = {FREQ ,SCLK, MOSI, MISO, 5, 6, 7};
const struct spi_setup SPI_F3 = {FREQ ,SCLK, MOSI, MISO, 12, 13, 14};
const struct spi_setup SPI_F4 = {FREQ ,SCLK, MOSI, MISO, 15, 16, 17};
const struct spi_setup SPI_F5 = {FREQ ,SCLK, MOSI, MISO, 18, 19, 20};
//There is enough GPIO pins for a total of 7 fingers.
//Anything above two fingers will not be tested.
//Only cartesian and joint space controllers have support for more than 2 fingers.
//This is because the fingers are not controlled induvidually in the other controllers

pthread_t tid[6];

static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;


typedef struct spi{
	unsigned handle;
	spi_setup setup;
	char outBuf[4];
	char inBuf[4];
}spi;


//Variables used by joint space PID function
typedef struct jointspace_pid_var {
   char read_angle_cmd[2];
   unsigned char inBuf[2];
   short theta1;
   short theta2;
   short error1;
   short error2;
   short theta1_setpoint;
   short theta2_setpoint;
   double kp1;
   double kp2;
   short u1;
   short u2;
   char run;
}jointspace_pid_var;

//Variables used by cartesian PID function
typedef struct cartesian_pid_var {
   char read_angle_cmd[2];
   unsigned char inBuf[2];
   short theta1;
   short theta2;
   short error1;
   short error2;
   short theta1_setpoint;
   short theta2_setpoint;
   double kp1;
   double kp2;
   short u1;
   short u2;
   char run;
}cartesian_pid_var;

typedef struct controller_variables{
	cartesian_pid_var	 cs;
	jointspace_pid_var js;
}controller_variables;

typedef struct zmq_payload{
	short data1;
	short data2;
	short data3;
	short data4;
	short data5;
}zmq_payload;

typedef struct zmq_instructions{
	void (*controller)(void*, void*);
	zmq_payload payload;
}zmq_instructions;

typedef struct finger_data{
	zmq_instructions*  zmq;		             //ptr to shared memory
	zmq_instructions   zmq_local;	   		   //work memory
	controller_variables controller_var;	 //variables used by controllers
  spi spi_data;                          //Constants used for SPI
}finger_data;

typedef struct zmq_data{
	char* address;
	char* contents;
	void* context;
	void* subscriber;
	char function_flag;
	zmq_instructions instr_finger1;
	zmq_instructions instr_finger2;
	zmq_instructions instr_finger3;
	zmq_instructions instr_finger4;
	zmq_instructions instr_finger5;
	int finger_select;
	int controller_select;
  void* controller_ptr[4];
}zmq_data;

void* read_zmq_server(void* zmq_read_input){

  //Casting input
	zmq_data* zmq_read = (zmq_data*) zmq_read_input;
	while (1) {

    zmq_read->address = s_recv (zmq_read->subscriber);  //  Read envelope with address
    zmq_read->contents = s_recv (zmq_read->subscriber); //  Read message contents


    pthread_mutex_lock(&lock);
  	//Analyse input
  	zmq_read->finger_select = zmq_read->contents[0] >> 4;				//4 MSB in first byte represent finger
  	zmq_read->controller_select = zmq_read->contents[0] & 00001111;			//4 LSB in first byte represent controller

  	//Store payload
         	if	( (zmq_read->finger_select) = 0b00000000){
  		sscanf(zmq_read->contents, "%*c %d %d %d %d %d"	, &(zmq_read->instr_finger1.payload.data1)
  						 		, &(zmq_read->instr_finger1.payload.data2)
  								, &(zmq_read->instr_finger1.payload.data3)
  								, &(zmq_read->instr_finger1.payload.data4)
  								, &(zmq_read->instr_finger1.payload.data5));
  		zmq_read->instr_finger1.controller = zmq_read->controller_ptr[zmq_read->controller_select];
  	}
  	else if	( (zmq_read->finger_select) = 0b00000001){
  		sscanf(zmq_read->contents, "%*c %d %d %d %d %d"	, &(zmq_read->instr_finger2.payload.data1)
  						 		, &(zmq_read->instr_finger2.payload.data2)
  								, &(zmq_read->instr_finger2.payload.data3)
  								, &(zmq_read->instr_finger2.payload.data4)
  								, &(zmq_read->instr_finger2.payload.data5));
  		zmq_read->instr_finger1.controller = zmq_read->controller_ptr[zmq_read->controller_select];
  	}
  	else if	( (zmq_read->finger_select) == 0b00000010){
  		sscanf(zmq_read->contents, "%*c %d %d %d %d %d"	, &(zmq_read->instr_finger3.payload.data1)
  						 		, &(zmq_read->instr_finger3.payload.data2)
  								, &(zmq_read->instr_finger3.payload.data3)
  								, &(zmq_read->instr_finger3.payload.data4)
  								, &(zmq_read->instr_finger3.payload.data5));
  		zmq_read->instr_finger1.controller = zmq_read->controller_ptr[zmq_read->controller_select];
  	}
  	else if	( (zmq_read->finger_select) == 0b00000011){
  		sscanf(zmq_read->contents, "%*c %d %d %d %d %d"	, &(zmq_read->instr_finger4.payload.data1)
  						 		, &(zmq_read->instr_finger4.payload.data2)
  								, &(zmq_read->instr_finger4.payload.data3)
  								, &(zmq_read->instr_finger4.payload.data4)
  								, &(zmq_read->instr_finger4.payload.data5));
  		zmq_read->instr_finger1.controller = zmq_read->controller_ptr[zmq_read->controller_select];
  	}
  	else if	( (zmq_read->finger_select) == 0b00000100){
  		sscanf(zmq_read->contents, "%*c %d %d %d %d %d"	, &(zmq_read->instr_finger5.payload.data1)
  						 		, &(zmq_read->instr_finger5.payload.data2)
  								, &(zmq_read->instr_finger5.payload.data3)
  								, &(zmq_read->instr_finger5.payload.data4)
  								, &(zmq_read->instr_finger5.payload.data5));
  		zmq_read->instr_finger1.controller = zmq_read->controller_ptr[zmq_read->controller_select];
  	}

    free (zmq_read->address);
    free (zmq_read->contents);
    pthread_mutex_unlock(&lock);
  	usleep(1000);
    }
}


void calibration(void* payload, void* vars, void* spi_);

void jointspace_pid(void* payload_in, void* vars, void* spi_){
  //Casting input
  zmq_payload* payload = (zmq_payload*) payload_in;
  controller_variables* controller_vars = (controller_variables*) vars;
	spi* spi = (spi*) spi_;
  //Run controller
  //Read SPI SENSORS
	/*
  controller_vars->js.theta1 = 50;        //For thesting without sensor (Remove this) CAST TO SHORT ON REAL IMPLEMENTATION
  controller_vars->js.theta2 = 50;        //For thesting without sensor (Remove this)
  /*
  bbSPIXfer(link1, read_angle_cmd, (char *)inBuf, 1); // > DAC
  theta1=inBuf[0];
  error1= (int short)( ((uint8_t) zmq_read->link1_angle)-inBuf[0]);
  bbSPIXfer(link2, read_angle_cmd, (char *)inBuf, 1); // > DAC
  theta2=inBuf[0];
	*/
	int spi_result;
	spi->outBuf[0] = 0b00000000;
	gpioWrite(spi->setup.cs_angle_sensor_1,0);
	spi_result = spiXfer(spi->handle, spi->outBuf, spi->inBuf, 1);
	gpioWrite(spi->setup.cs_angle_sensor_1,1);
	controller_vars->js.theta1 = inBuf[0];

	spi->outBuf[0] = 0b00000000;
	gpioWrite(spi->setup.cs_angle_sensor_1,0);
	spi_result = spiXfer(spi->handle, spi->outBuf, spi->inBuf, 1);
	gpioWrite(spi->setup.cs_angle_sensor_1,1);
	controller_vars->js.theta2 = spi->inBuf[0];

  printf("Jointspace\n");
  controller_vars->js.theta1_setpoint = payload->data1;
  controller_vars->js.theta2_setpoint = payload->data2;
  //Proportional controller
  controller_vars->js.error1 = (controller_vars->js.theta1_setpoint - controller_vars->js.theta1);
  controller_vars->js.u1 = (short) (controller_vars->js.error1*controller_vars->js.kp1);
  controller_vars->js.error2 = (controller_vars->js.theta2_setpoint - controller_vars->js.theta2);
  controller_vars->js.u2 = (short) (controller_vars->js.error2*controller_vars->js.kp2);
  printf("theta1: %d | theta1_setpoint: %d | error1: %d | u1: %d \n", controller_vars->js.theta1 , controller_vars->js.theta1_setpoint, controller_vars->js.error1, controller_vars->js.u1);
  printf("theta2: %d | theta2_setpoint: %d | error2: %d | u2: %d \n", controller_vars->js.theta2 , controller_vars->js.theta2_setpoint, controller_vars->js.error2, controller_vars->js.u2);

  //Write to ESP32 through SPI
  usleep(1000);
}

void cartesian_pid_controller(void* payload_in, void* vars, void* pid_){
  //Casting input
  zmq_payload* payload = (zmq_payload*) payload_in;
  controller_variables* controller_vars = (controller_variables*) vars;

  //Run controller
    /*

    //Read input, check if controller is still selected (Shared resources)
    pthread_mutex_lock(&lock);
    //INSERT READINGS HERE
    pid_var->run = (pid_var->zmq_bundle->function_flag == 0b00000010);
    pthread_mutex_unlock(&lock);
    if (!pid_var->run){ //Input not relevant for this controller
      break;            //Try next controller
    }*/
    printf("Cartesian\n");

    /*
    Insert controller here
    */

    usleep(1000);

}
void* no_controller(void* a, void* b, void* c){
   //Empty controller. used when nothing happens
}
/*
void controller_select(void* jointspace_pid_var, void* cartesian_pid_var){
  while(1){
    jointspace_pid(jointspace_pid_var);
    cartesian_pid_controller(cartesian_pid_var);
  }
}
*/

void* run_controller_once(finger_data* finger){

	//READ ZMQ INSTRUCTION FROM SHARED MEMORY.
  pthread_mutex_lock(&lock);
	finger->zmq_local = *(finger->zmq);
  pthread_mutex_unlock(&lock);

	//Execute instructions
	(*finger->zmq_local.controller)(&finger->zmq_local.payload, &finger->controller_var, &finger->spi_data); //Runninng a pointer to a controller function
}

finger_controllers(zmq_data* shared){

	//Initialize 5 finger containers
	finger_data f1_data;
	finger_data f2_data;
	finger_data f3_data;
	finger_data f4_data;
	finger_data f5_data;

	//Set SPI options
	f1_data.spi_data.setup = SPI_F1;
	f2_data.spi_data.setup = SPI_F2;
	f3_data.spi_data.setup = SPI_F3;
	f4_data.spi_data.setup = SPI_F4;
	f5_data.spi_data.setup = SPI_F5;

	//Add pointers to the relevant input data from zmq
	f1_data.zmq = &(shared->instr_finger1);
	f2_data.zmq = &(shared->instr_finger2);
	f3_data.zmq = &(shared->instr_finger3);
	f4_data.zmq = &(shared->instr_finger4);
	f5_data.zmq = &(shared->instr_finger5);

  //Set default settings for controllers
  controller_variables default_cntrl_set;             //Creating controller set
  default_cntrl_set.js.kp1 = 1;                        //joint space controller vaules
  default_cntrl_set.js.kp2 = 0.5;                      //joint space controller vaules
  default_cntrl_set.js.read_angle_cmd[0] = 0b00000000; //SPI command
  default_cntrl_set.js.read_angle_cmd[1] = 0b00000000; //SPI command
  default_cntrl_set.cs.kp1 = 1;                        //cartesian space controller values
  default_cntrl_set.cs.kp2 = 0.5;                      //cartesian space controller values
  default_cntrl_set.cs.read_angle_cmd[0] = 0b00000000; //SPI command
  default_cntrl_set.cs.read_angle_cmd[1] = 0b00000000; //SPI command

  f1_data.controller_var = default_cntrl_set;
  f2_data.controller_var = default_cntrl_set;
  f3_data.controller_var = default_cntrl_set;
  f4_data.controller_var = default_cntrl_set;
  f5_data.controller_var = default_cntrl_set;

  //Select an empty controller by default_widget
  f1_data.zmq_local.controller = no_controller;
  f2_data.zmq_local.controller = no_controller;
  f3_data.zmq_local.controller = no_controller;
  f4_data.zmq_local.controller = no_controller;
  f5_data.zmq_local.controller = no_controller;

	//Set SPI chip selects high by default (No chip selected)
	gpioWrite(f1_data.spi_data.setup.cs_angle_sensor_1, 1);
	gpioWrite(f1_data.spi_data.setup.cs_angle_sensor_2, 1);
	gpioWrite(f1_data.spi_data.setup.cs_esp32, 1);
	gpioWrite(f2_data.spi_data.setup.cs_angle_sensor_1, 1);
	gpioWrite(f2_data.spi_data.setup.cs_angle_sensor_2, 1);
	gpioWrite(f2_data.spi_data.setup.cs_esp32, 1);
	gpioWrite(f3_data.spi_data.setup.cs_angle_sensor_1, 1);
	gpioWrite(f3_data.spi_data.setup.cs_angle_sensor_2, 1);
	gpioWrite(f3_data.spi_data.setup.cs_esp32, 1);
	gpioWrite(f4_data.spi_data.setup.cs_angle_sensor_1, 1);
	gpioWrite(f4_data.spi_data.setup.cs_angle_sensor_2, 1);
	gpioWrite(f4_data.spi_data.setup.cs_esp32, 1);
	gpioWrite(f5_data.spi_data.setup.cs_angle_sensor_1, 1);
	gpioWrite(f5_data.spi_data.setup.cs_angle_sensor_2, 1);
	gpioWrite(f5_data.spi_data.setup.cs_esp32, 1);

	//SPI handle
	int spi_handle = spiOpen(CS0, FREQ, 0);
	if (spi_handle < 0)
	{
		fprintf("SPI OPEN FAILED\n");
			return 1;
	}
	f1_data.spi_data.handle = spi_handle;
	f2_data.spi_data.handle = spi_handle;
	f3_data.spi_data.handle = spi_handle;
	f4_data.spi_data.handle = spi_handle;
	f5_data.spi_data.handle = spi_handle;

	//Separate into 5 threads that runs the controller once. The controllers are synched up by joining the threads.
	while(1){

  	pthread_create(&(tid[1]), NULL, run_controller_once, &f1_data);
		pthread_create(&(tid[2]), NULL, run_controller_once, &f2_data);
		pthread_create(&(tid[3]), NULL, run_controller_once, &f3_data);
		pthread_create(&(tid[4]), NULL, run_controller_once, &f4_data);
		pthread_create(&(tid[5]), NULL, run_controller_once, &f5_data);

		pthread_join(tid[1], NULL);
  	pthread_join(tid[2], NULL);
  	pthread_join(tid[3], NULL);
  	pthread_join(tid[4], NULL);
 		pthread_join(tid[5], NULL);

	}
}

int main()
{

  //VARIABLES USED IN FUNCTIONS
  zmq_data zmq_var;
  //  Prepare our context and subscriber
  zmq_var.context = zmq_ctx_new ();
  zmq_var.subscriber = zmq_socket (zmq_var.context, ZMQ_SUB);
  //Assigning controllers to choose from
  zmq_var.controller_ptr[0] = &jointspace_pid;
  zmq_var.controller_ptr[1] = &cartesian_pid_controller;
  zmq_var.controller_ptr[2] = no_controller;
  zmq_var.controller_ptr[3] = no_controller;
  //Default controller at startup for each finger
  zmq_var.instr_finger1.controller = no_controller;
  zmq_var.instr_finger2.controller = no_controller;
  zmq_var.instr_finger3.controller = no_controller;
  zmq_var.instr_finger4.controller = no_controller;
  zmq_var.instr_finger5.controller = no_controller;




  printf("%p\n",(void*) zmq_var.controller_ptr[0]);
  printf("%p\n",(void*) zmq_var.controller_ptr[1]);
  printf("%p\n",(void*) zmq_var.controller_ptr[2]);
  printf("%p\n",(void*) zmq_var.controller_ptr[3]);

	//SPI INIT
	if (gpioInitialise() < 0)
	{
		 fprintf(stderr, "pigpio initialisation failed.\n");
		 return 1;
	}
	int spi_handle = spiOpen(CS0, FREQ, CS9, 0);
	if (spi_handle < 0)
	{
		fprintf("SPI OPEN FAILED\n");
			return 1;
	}

 /*
  jointspace_pid_var j_pid_var;
  //j_pid_var.zmq_bundle = &zmq_var;
  j_pid_var.read_angle_cmd[0] = 0b00000000; //SPI command
  j_pid_var.read_angle_cmd[1] = 0b00000000; //SPI command
  j_pid_var.kp1 = 1;
  j_pid_var.kp2 = 0.5;

  cartesian_pid_var c_pid_var;
  //c_pid_var.zmq_bundle = &zmq_var;
  c_pid_var.read_angle_cmd[0] = 0b00000000; //SPI command
  c_pid_var.read_angle_cmd[1] = 0b00000000; //SPI command
  c_pid_var.kp1 = 1;
  c_pid_var.kp2 = 0.5;
*/
  //void *context = zmq_ctx_new ();
  //void *subscriber = zmq_socket (context, ZMQ_SUB);
  //zmq_connect (subscrpid_var->iber, "tcp://10.218.130.229:5563");
  zmq_connect (zmq_var.subscriber, "tcp://localhost:5563");
  zmq_setsockopt (zmq_var.subscriber, ZMQ_SUBSCRIBE, "B", 1);


  //ZMQ thread:
  pthread_create(&(tid[0]), NULL, &read_zmq_server, &zmq_var);
  usleep(1000);

  finger_controllers(&zmq_var);
  //controller_select(&mc_pid_var, &j_pid_var);
  //pthread_create(&(tid[1]), NULL, &pid, &pid_var);
  pthread_join(tid[0], NULL);
  pthread_join(tid[1], NULL);
  pthread_join(tid[2], NULL);
  pthread_join(tid[3], NULL);
  pthread_join(tid[4], NULL);
  pthread_join(tid[5], NULL);
  //pthread_join(tid[1], NULL);

  //pthread_create(&(tid[0]), NULL, &read_reference_angle, &zmq_read);

  /*


   int count, set_val, read_val, x, SPI_init1, SPI_init2, SPI_init3, cout_itr=1;
   unsigned char inBuf[4];
   char read_angle_cmd[]= {0b00000000, 0b00000000};
   char set_zero_angle_cmd[2];
   char torque_cmd[4];
   uint16_t zero_point;

   //PID
   double kp1=0.1;
   double kp2=0.1;
   double max_torque1=0.3;    //unused
   double max_torque2=0.15;   //unused


   uint8_t theta1;
   uint8_t theta2;
   uint8_t theta1_0;    //Angle bias link 1
   uint8_t theta2_0;    //Angle bias link 2
   uint8_t setpoint1 = 42; //Set point at approx 60 deg from end position
   uint8_t setpoint2 = 42;

   int short error1;
   int short error2;
   int short u1;
   int short u2;


   //INIT
   if (gpioInitialise() < 0)
   {
      fprintf(stderr, "pigpio initialisation failed.\n");
      return 1;
   }

   */
	 if (spiClose(spi_handle) < 0){
		 printf("Bad handle");
	 }
	 gpioTerminate();
}



void calibration(void* payload, void* vars,  void* spi_){
	//Casting input
	zmq_payload* payload = (zmq_payload*) payload_in;
	controller_variables* controller_vars = (controller_variables*) vars;
	spi* spi = (spi*) spi_;

	char read_angle_cmd[]= {0b00000000, 0b00000000};
	char set_zero_angle_cmd[2];
	char torque_cmd[4];
	uint16_t zero_point;

	//******DRIVE MOTORS TO END POSITION*******
	//*****************************************
	//NOT DONE
	torque_cmd[0]=(uint8_t) 0;
	torque_cmd[1]=(uint8_t) 20;
	torque_cmd[2]=(uint8_t) 0;
	torque_cmd[3]=(uint8_t) 20;

	printf("Driving to endpoint");
	usleep(10000000);

	//READ ANGLE AT END POINT
	int spi_result;
	spi->outBuf[0] = 0b00000000;
	gpioWrite(spi->setup.cs_angle_sensor_1,0);
	spi_result = spiXfer(spi->handle, spi->outBuf, spi->inBuf, 1);
	gpioWrite(spi->setup.cs_angle_sensor_1,1):
	controller_vars->js.theta1 = inBuf[0];

	spi->outBuf[0] = 0b00000000;
	gpioWrite(spi->setup.cs_angle_sensor_1,0);
	spi_result = spiXfer(spi->handle, spi->outBuf, spi->inBuf, 1);
	gpioWrite(spi->setup.cs_angle_sensor_1,1);
	controller_vars->js.theta2 = inBuf[0];

	printf("Before calibration: %d | %d", controller_vars->js.theta1 , 	controller_vars->js.theta2);
	//Setting zero_angle at start position
	//The measured angle in end position should be zero to avoid crossing from 0->255, as this will mess with the PID.
	//Any previous zero angle setting is removed before the angle is measured. This measured angle is set as the new zero angle.
	//A delay followed by 16 zeros is required after each write to sensor register.
	//The register value should be the compliment of the wanted zero-angle


	//*********CALIBRATE SENSOR 1**********
	//*************************************

	//RESET OLD ZERO POINT
	set_zero_angle_cmd[0]=0b10000001; //WRITE REG 1 (8 MSB of zero angle)
	set_zero_angle_cmd[1]=0b00000000; //ZERO-ANGLE SET TO 0
	gpioWrite(spi->setup.cs_angle_sensor_1,0);
	spi_result = spiXfer(spi->handle, set_zero_angle_cmd, spi->inBuf, 2);
	gpioWrite(spi->setup.cs_angle_sensor_1,1);
	usleep(10000);
	gpioWrite(spi->setup.cs_angle_sensor_1,0);
	spi_result = spiXfer(spi->handle, read_angle_cmd, spi->inBuf, 2);
	gpioWrite(spi->setup.cs_angle_sensor_1,1);
	usleep(10000);
	set_zero_angle_cmd[0]=0b10000000; //WRITE REG 0 (8 LSB of zero angle)
	set_zero_angle_cmd[1]=0b00000000; //ZERO-ANGLE SET TO 0
	gpioWrite(spi->setup.cs_angle_sensor_1,0);
	spi_result = spiXfer(spi->handle, set_zero_angle_cmd, spi->inBuf, 2);
	gpioWrite(spi->setup.cs_angle_sensor_1,1);
	usleep(10000);
	gpioWrite(spi->setup.cs_angle_sensor_1,0);
	spi_result = spiXfer(spi->handle, read_angle_cmd, spi->inBuf, 2);
	gpioWrite(spi->setup.cs_angle_sensor_1,1);
	usleep(10000);

	//MEASURE ANGLE AFTER RESET AND CALCULATE REGISTER INPUT
	gpioWrite(spi->setup.cs_angle_sensor_1,0);
	spi_result = spiXfer(spi->handle, read_angle_cmd, spi->inBuf, 2);
	gpioWrite(spi->setup.cs_angle_sensor_1,1);									//MEASURE CURRENT ANGLE
	zero_point = (inBuf[0] << 8);                               //COMBINE 8 bit values to 16 bit
	zero_point = zero_point + inBuf[1];
	zero_point = (uint16_t) (0b10000000000000000-zero_point);   //CALCULATE COMPLIMENT (Formula 4 in Datasheet:  MagAlpha MA302  12-Bit, Digital, Contactless Angle Sensor with ABZ & UVW Incremental Outputs )

	//SET NEW ZERO POINT
	set_zero_angle_cmd[0]=0b10000001;
	set_zero_angle_cmd[1]=(uint8_t) (zero_point >> 8);          //8 MSB of Compliment of new zero angle
	gpioWrite(spi->setup.cs_angle_sensor_1,0);
	spi_result = spiXfer(spi->handle, set_zero_angle_cmd, spi->inBuf, 2);
	gpioWrite(spi->setup.cs_angle_sensor_1,1);
	usleep(50000);
	gpioWrite(spi->setup.cs_angle_sensor_1,0);
	spi_result = spiXfer(spi->handle, read_angle_cmd, spi->inBuf, 2);
	gpioWrite(spi->setup.cs_angle_sensor_1,1);
	usleep(50000);
	set_zero_angle_cmd[0]=0b10000000;
	set_zero_angle_cmd[1]=(uint8_t) zero_point;                 //8 LSB of Compliment of new zero angle
	gpioWrite(spi->setup.cs_angle_sensor_1,0);
	spi_result = spiXfer(spi->handle, set_zero_angle_cmd, spi->inBuf, 2);
	gpioWrite(spi->setup.cs_angle_sensor_1,1);
	usleep(50000);
	gpioWrite(spi->setup.cs_angle_sensor_1,0);
	spi_result = spiXfer(spi->handle, read_angle_cmd, spi->inBuf, 2);
	gpioWrite(spi->setup.cs_angle_sensor_1,1);
	usleep(50000);



	//*********CALIBRATE SENSOR 2**********
	//*************************************

	//RESET OLD ZERO POINT
	set_zero_angle_cmd[0]=0b10000001;   //WRITE REG 1 (8 MSB of zero angle)
	set_zero_angle_cmd[1]=0b00000000;   //RESET ZERO ANGLE
	gpioWrite(spi->setup.cs_angle_sensor_2,0);
	spi_result = spiXfer(spi->handle, set_zero_angle_cmd, spi->inBuf, 2);
	gpioWrite(spi->setup.cs_angle_sensor_2,1);
	usleep(50000);
	gpioWrite(spi->setup.cs_angle_sensor_2,0);
	spi_result = spiXfer(spi->handle, read_angle_cmd, spi->inBuf, 2);
	gpioWrite(spi->setup.cs_angle_sensor_2,1);
	usleep(50000);
	set_zero_angle_cmd[0]=0b10000000;   //WRITE REG 0 (8 LSB of zero angle)
	set_zero_angle_cmd[1]=0b00000000;   //RESET ZERO ANGLE
	gpioWrite(spi->setup.cs_angle_sensor_2,0);
	spi_result = spiXfer(spi->handle, set_zero_angle_cmd, spi->inBuf, 2);
	gpioWrite(spi->setup.cs_angle_sensor_2,1);
	usleep(50000);
	gpioWrite(spi->setup.cs_angle_sensor_2,0);
	spi_result = spiXfer(spi->handle, read_angle_cmd, spi->inBuf, 2);
	gpioWrite(spi->setup.cs_angle_sensor_2,1);
	usleep(50000);

	//MEASURE ANGLE AFTER RESET AND CALCULATE REGISTER INPUT
	gpioWrite(spi->setup.cs_angle_sensor_2,0);
	spi_result = spiXfer(spi->handle, read_angle_cmd, spi->inBuf, 2);
	gpioWrite(spi->setup.cs_angle_sensor_2,1); // MEASURE ZERO ANGLE
	zero_point = (inBuf[0] << 8);
	zero_point = zero_point + inBuf[1];
	zero_point = (uint16_t) (0b10000000000000000-zero_point);   //CALCULATE COMPLIMENT (Formula 4 in Datasheet:  MagAlpha MA302  12-Bit, Digital, Contactless Angle Sensor with ABZ & UVW Incremental Outputs )
	set_zero_angle_cmd[0]=0b10000001;                           //WRITE REG 1 (8 MSB of zero angle)
	set_zero_angle_cmd[1]=(uint8_t) (zero_point >> 8);          //ZERO ANGLE SET TO CURRENT ANGLE

	//SET NEW ZERO POINT
	gpioWrite(spi->setup.cs_angle_sensor_2,0);
	spi_result = spiXfer(spi->handle, set_zero_angle_cmd, spi->inBuf, 2);
	gpioWrite(spi->setup.cs_angle_sensor_2,1);
	usleep(50000);
	gpioWrite(spi->setup.cs_angle_sensor_2,0);
	spi_result = spiXfer(spi->handle, read_angle_cmd, spi->inBuf, 2);
	gpioWrite(spi->setup.cs_angle_sensor_2,1);
	usleep(50000);
	set_zero_angle_cmd[0]=0b10000000;                           //WRITE REG 0 (8 LSB of zero angle)
	set_zero_angle_cmd[1]=(uint8_t) zero_point;                 //ZERO ANGLE SET TO CURRENT ANGLE
	gpioWrite(spi->setup.cs_angle_sensor_2,0);
	spi_result = spiXfer(spi->handle, set_zero_angle_cmd, spi->inBuf, 2);
	gpioWrite(spi->setup.cs_angle_sensor_2,1);
	usleep(50000);
	gpioWrite(spi->setup.cs_angle_sensor_2,0);
	spi_result = spiXfer(spi->handle, read_angle_cmd, spi->inBuf, 2);
	gpioWrite(spi->setup.cs_angle_sensor_2,1);
	usleep(50000);


	//*********PRINT RESULT****************
	//*************************************
	gpioWrite(spi->setup.cs_angle_sensor_1,0);
	spi_result = spiXfer(spi->handle, read_angle_cmd, spi->inBuf, 1);
	gpioWrite(spi->setup.cs_angle_sensor_1,1);
	controller_vars->js.theta1=inBuf[0];

	gpioWrite(spi->setup.cs_angle_sensor_2,0);
	spi_result = spiXfer(spi->handle, read_angle_cmd, spi->inBuf, 1);
	gpioWrite(spi->setup.cs_angle_sensor_2,1);
	controller_vars->js.theta2=inBuf[0];

	printf("After calibration: %d | %d \n",	controller_vars->js.theta1, controller_vars->js.theta2 );
}
