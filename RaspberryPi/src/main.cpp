
#include <math.h>
#include "zhelpers.h"
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
pthread_t tid[10];
static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;


//Variables used by joint space PID function
typedef struct jointspace_pid_var {
   double error1;
   double error2;
   double* theta1_setpoint;
   double* theta2_setpoint;
   double kp1;
	 double ki1;
	 double kd1;
   double kp2;
	 double ki2;
	 double kd2;
}jointspace_pid_var;

//Variables used by cartesian PID function
typedef struct cartesian_pid_var {
   double error1;
   double error2;
   double theta1_setpoint;
   double theta2_setpoint;
	 double kp1;
	 double ki1;
	 double kd1;
   double kp2;
	 double ki2;
	 double kd2;
	 double temp;
	 double k1;
	 double k2;
	 double gamma;
	 double l1;
	 double l2;
	 double* x;
	 double* y;
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
	void (*controller)(void*, void*, void*);
	zmq_payload payload;
}zmq_instructions;

typedef struct finger_data{
	zmq_instructions*  zmq;		             //ptr to shared memory
	zmq_instructions   zmq_local;	   		   //work memory
	controller_variables controller_var;	 //variables used by controllers                        //Constants used for SPI
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


class finger{
  public:
		//void update_local_zmq_mem();					//updates controller_select and data1-4
	//	void update_local_spi_mem();					//updates theta1/2 and angular_vel1/2
	//	void update_shared_spi_mem();					//updates torque1/2 on shared spi memory
	//	void shutdown();											//sets runflag to zero for both spi and zmq on shared memory

		//Runs on startup. Sets zero angle on sensors.
		//void calibration();

		//Controllers
	//	void cartesian_ijc_pid();							 //Independant joint controller with cartesian input
	//	void jointspace_ijc_pid();						 //Independant joint controller with joint_angle input

		//Main loop
	//	void run();

		//Define a set of variables used by the PID idependant joint controller
		//with cartesian coordinates as input
		cartesian_pid_var pid_ijc_cs;
		//Define a set of variables used by the PID idependant joint controller
		//with joint angle set point as input
		jointspace_pid_var pid_ijc_js;

 		//MEMORY SHARED WITH ZMQ Thread
		double* zmq_mem_shared;
		//LOCAL BUFFER OF SHARED ZMQ Memory
		double runflag_zmq;
		double controller_select;
		double data1;
		double data2;
		double data3;
		double data4;

		//Memory shared with spi thread
		double* spi_mem_shared;
		//Local buffer of shared spi memory
		double runflag_spi;
		double theta1;
		double theta2;
		double angular_vel1;
		double angular_vel2;
		double torque1;
		double torque2;

		//count controller cycles
		int itr_counter;
		int id;

		//SPI variables(Used only for calibration)
		int cs_angle_sensor_1;
		int cs_angle_sensor_2;
		int cs_output;
		int handle;
		int frequency;
		int spi_channel;
		int sclk;
		int mosi;
		int miso;
		char inBuf[4];
		char outBuf[4];

		//Constructor
    finger(int identity ,double shared_spi_memory[7], double shared_zmq_memory[6], int spi_var[4]){
			id= identity;
			//Get pointers to shared memory
			spi_mem_shared = shared_spi_memory;
			zmq_mem_shared = shared_zmq_memory;


      //Set default settings for controllers
			//joint space controller vaules
      pid_ijc_js.kp1 = 1;
		  pid_ijc_js.ki1 = 0;
			pid_ijc_js.kd1 = 0;
			pid_ijc_js.kp2 = 0.5;
			pid_ijc_js.ki2 = 0;
			pid_ijc_js.kd2 = 0;
			//cartesian space controller values
      pid_ijc_cs.kp1 = 1;
      pid_ijc_cs.ki1 = 0;
      pid_ijc_cs.kd1 = 0;
			pid_ijc_cs.kp2 = 0.5;
      pid_ijc_cs.ki2 = 0;
      pid_ijc_cs.kd2 = 0;
      pid_ijc_cs.l1 = 5.3;
      pid_ijc_cs.l2 = 4.8;

			//Pointing controller variables to local zmq buffer
			//(Data1-4 has potentially different meanings to different controllers)
			pid_ijc_js.theta1_setpoint = &data1;
			pid_ijc_js.theta2_setpoint = &data2;

			pid_ijc_cs.x = &data1;
			pid_ijc_cs.y = &data2;

			itr_counter=0;

			//SPI:
			//During normal operation, only the dedicated spi-thread talks to the sensors,
			//However, since
			frequency = 15000000;
			spi_channel = 0;
			sclk = 11;
			mosi = 10;
			miso = 9;
			cs_angle_sensor_1 = spi_var[0];
			cs_angle_sensor_2 = spi_var[1];
			cs_output = spi_var[2];
			handle = spi_var[3];
    }

		void shutdown(){
			//Tell zmq client that the thread is no longer active
      std::cout << "shutting down thread: " << id <<std::endl;
			pthread_mutex_lock(&lock);
			spi_mem_shared[0] = 0;
			pthread_mutex_unlock(&lock);
			//Tell zmq function to no longer measure sesnors for this finger
			pthread_mutex_lock(&lock);
			zmq_mem_shared[0] = 0;
			pthread_mutex_unlock(&lock);
			}

		void update_local_zmq_mem(){
			pthread_mutex_lock(&lock);
			controller_select = zmq_mem_shared[1];
			data1 = zmq_mem_shared[2];
			data2 = zmq_mem_shared[3];
			data3 = zmq_mem_shared[4];
			data4 = zmq_mem_shared[5];
		}

		void update_local_spi_mem(){
			pthread_mutex_lock(&lock);
			theta1 = spi_mem_shared[1];
			theta2 = spi_mem_shared[2];
			angular_vel1 = spi_mem_shared[3];
			angular_vel2 = spi_mem_shared[4];
			pthread_mutex_unlock(&lock);
		}

		void update_shared_spi_mem(){
			pthread_mutex_lock(&lock);
			spi_mem_shared[5]=torque1;
			spi_mem_shared[6]=torque2;
			pthread_mutex_unlock(&lock);
		}

		void calibration(){

			std::cout << "Hold on, im calibrating finger " << id << std::endl;
			char read_angle_cmd[]= {0b00000000, 0b00000000, 0b00000000};
			char set_zero_angle_cmd[2];
			char torque_cmd[3];
			uint16_t zero_point;
			int gpio_result;
			int spi_result;

			//******DRIVE MOTORS TO END POSITION*******
			//*****************************************
			//NOT DONE
			torque_cmd[0]=(uint8_t) 0;
			torque_cmd[1]=(uint8_t) 20;
			torque_cmd[2]=(uint8_t) 20;

      			std::cout << "about to use spi " << std::endl;
			pthread_mutex_lock(&lock);
			gpio_result = gpioWrite(cs_output,0);
			spi_result = spiXfer(handle, torque_cmd, inBuf, 3);
			gpio_result = gpioWrite(cs_output,1);
			pthread_mutex_unlock(&lock);

			printf("Driving to endpoint\n");
			usleep(5000000);

			//READ ANGLE AT END POINT
			outBuf[0] = 0b00000000;
			pthread_mutex_lock(&lock);
			gpio_result = gpioWrite(cs_angle_sensor_1,0);
			spi_result = spiXfer(handle, outBuf, inBuf, 1);
			gpio_result = gpioWrite(cs_angle_sensor_1,1);
			pthread_mutex_unlock(&lock);
			theta1 = inBuf[0];

			outBuf[0] = 0b00000000;
			pthread_mutex_lock(&lock);
			gpio_result = gpioWrite(cs_angle_sensor_2,0);
			spi_result = spiXfer(handle, outBuf, inBuf, 1);
			gpio_result = gpioWrite(cs_angle_sensor_2,1);
		 	pthread_mutex_unlock(&lock);
			theta2 = inBuf[0];

			printf("Before calibration: %d | %d\n", theta1 , theta2);
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
			pthread_mutex_lock(&lock);
			gpio_result = gpioWrite(cs_angle_sensor_1,0);
			spi_result = spiXfer(handle, set_zero_angle_cmd, inBuf, 2);
			gpio_result = gpioWrite(cs_angle_sensor_1,1);
			pthread_mutex_unlock(&lock);
			usleep(100000);
			pthread_mutex_lock(&lock);
			gpio_result = gpioWrite(cs_angle_sensor_1,0);
			spi_result = spiXfer(handle, read_angle_cmd, inBuf, 2);
			gpio_result = gpioWrite(cs_angle_sensor_1,1);
			pthread_mutex_unlock(&lock);
			usleep(100000);
			set_zero_angle_cmd[0]=0b10000000; //WRITE REG 0 (8 LSB of zero angle)
			set_zero_angle_cmd[1]=0b00000000; //ZERO-ANGLE SET TO 0
			pthread_mutex_lock(&lock);
			gpio_result = gpioWrite(cs_angle_sensor_1,0);
			spi_result = spiXfer(handle, set_zero_angle_cmd, inBuf, 2);
			gpio_result = gpioWrite(cs_angle_sensor_1,1);
			pthread_mutex_unlock(&lock);
			usleep(100000);
			pthread_mutex_lock(&lock);
			gpio_result = gpioWrite(cs_angle_sensor_1,0);
			spi_result = spiXfer(handle, read_angle_cmd, inBuf, 2);
			gpio_result = gpioWrite(cs_angle_sensor_1,1);
			pthread_mutex_unlock(&lock);
			usleep(100000);

			//MEASURE ANGLE AFTER RESET AND CALCULATE REGISTER INPUT
			pthread_mutex_lock(&lock);
			gpio_result = gpioWrite(cs_angle_sensor_1,0);
			spi_result = spiXfer(handle, read_angle_cmd, inBuf, 2);
			gpio_result = gpioWrite(cs_angle_sensor_1,1);									//MEASURE CURRENT ANGLE
			pthread_mutex_unlock(&lock);
			zero_point = (inBuf[0] << 8);                               //COMBINE 8 bit values to 16 bit
			zero_point = zero_point + inBuf[1];
			zero_point = (uint16_t) (0b10000000000000000-zero_point+0b0000100000000000);   	//CALCULATE COMPLIMENT (Formula 4 in Datasheet:  MagAlpha MA302  12-Bit, Digital, Contactless Angle Sensor with ABZ & UVW Incremental Outputs )
																																		 									//ADDING 8 (in 16bit) to avoid crossing zero because of noise
			//SET NEW ZERO POINT
			set_zero_angle_cmd[0]=0b10000001;
			set_zero_angle_cmd[1]=(uint8_t) (zero_point >> 8);          //8 MSB of Compliment of new zero angle
			pthread_mutex_lock(&lock);
			gpio_result = gpioWrite(cs_angle_sensor_1,0);
			spi_result = spiXfer(handle, set_zero_angle_cmd, inBuf, 2);
			gpio_result = gpioWrite(cs_angle_sensor_1,1);
			pthread_mutex_unlock(&lock);
			usleep(100000);
			pthread_mutex_lock(&lock);
			gpio_result = gpioWrite(cs_angle_sensor_1,0);
			spi_result = spiXfer(handle, read_angle_cmd, inBuf, 2);
			gpio_result = gpioWrite(cs_angle_sensor_1,1);
			pthread_mutex_unlock(&lock);
			usleep(100000);
			set_zero_angle_cmd[0]=0b10000000;
			set_zero_angle_cmd[1]=(uint8_t) zero_point;                 //8 LSB of Compliment of new zero angle
			pthread_mutex_lock(&lock);
			gpio_result = gpioWrite(cs_angle_sensor_1,0);
			spi_result = spiXfer(handle, set_zero_angle_cmd, inBuf, 2);
			gpio_result = gpioWrite(cs_angle_sensor_1,1);
			pthread_mutex_unlock(&lock);
			usleep(100000);
			pthread_mutex_lock(&lock);
			gpio_result = gpioWrite(cs_angle_sensor_1,0);
			spi_result = spiXfer(handle, read_angle_cmd, inBuf, 2);
			gpio_result = gpioWrite(cs_angle_sensor_1,1);
			pthread_mutex_unlock(&lock);
			usleep(100000);


			//*********CALIBRATE SENSOR 2**********
			//*************************************

			//RESET OLD ZERO POINT
			set_zero_angle_cmd[0]=0b10000001;   //WRITE REG 1 (8 MSB of zero angle)
			set_zero_angle_cmd[1]=0b00000000;   //RESET ZERO ANGLE
			pthread_mutex_lock(&lock);
			gpio_result = gpioWrite(cs_angle_sensor_2,0);
			spi_result = spiXfer(handle, set_zero_angle_cmd, inBuf, 2);
			gpio_result = gpioWrite(cs_angle_sensor_2,1);
			pthread_mutex_unlock(&lock);
			usleep(100000);
			pthread_mutex_lock(&lock);
			gpio_result = gpioWrite(cs_angle_sensor_2,0);
			spi_result = spiXfer(handle, read_angle_cmd, inBuf, 2);
			gpio_result = gpioWrite(cs_angle_sensor_2,1);
			pthread_mutex_unlock(&lock);
			usleep(100000);
			set_zero_angle_cmd[0]=0b10000000;   //WRITE REG 0 (8 LSB of zero angle)
			set_zero_angle_cmd[1]=0b00000000;   //RESET ZERO ANGLE
			pthread_mutex_lock(&lock);
			gpio_result = gpioWrite(cs_angle_sensor_2,0);
			spi_result = spiXfer(handle, set_zero_angle_cmd, inBuf, 2);
			gpio_result = gpioWrite(cs_angle_sensor_2,1);
			pthread_mutex_unlock(&lock);
			usleep(100000);
			pthread_mutex_lock(&lock);
			gpio_result = gpioWrite(cs_angle_sensor_2,0);
			spi_result = spiXfer(handle, read_angle_cmd, inBuf, 2);
			gpio_result = gpioWrite(cs_angle_sensor_2,1);
			pthread_mutex_unlock(&lock);
			usleep(100000);

			//MEASURE ANGLE AFTER RESET AND CALCULATE REGISTER INPUT
			pthread_mutex_lock(&lock);
			gpio_result = gpioWrite(cs_angle_sensor_2,0);
			spi_result = spiXfer(handle, read_angle_cmd, inBuf, 2);
			gpio_result = gpioWrite(cs_angle_sensor_2,1); // MEASURE ZERO ANGLE
			pthread_mutex_unlock(&lock);
			zero_point = (inBuf[0] << 8);
			zero_point = zero_point + inBuf[1];
			zero_point = (uint16_t) (0b10000000000000000-zero_point+0b0000100000000000);   //CALCULATE COMPLIMENT (Formula 4 in Datasheet:  MagAlpha MA302  12-Bit, Digital, Contactless Angle Sensor with ABZ & UVW Incremental Outputs )

			//SET NEW ZERO POINT
			set_zero_angle_cmd[0]=0b10000001;                           //WRITE REG 1 (8 MSB of zero angle)
			set_zero_angle_cmd[1]=(uint8_t) (zero_point >> 8);          //ZERO ANGLE SET TO CURRENT ANGLE
			pthread_mutex_lock(&lock);
			gpio_result = gpioWrite(cs_angle_sensor_2,0);
			spi_result = spiXfer(handle, set_zero_angle_cmd, inBuf, 2);
			gpio_result = gpioWrite(cs_angle_sensor_2,1);
			pthread_mutex_unlock(&lock);
			usleep(100000);
			pthread_mutex_lock(&lock);
			gpio_result = gpioWrite(cs_angle_sensor_2,0);
			spi_result = spiXfer(handle, read_angle_cmd, inBuf, 2);
			gpio_result = gpioWrite(cs_angle_sensor_2,1);
			pthread_mutex_unlock(&lock);
			usleep(100000);
			set_zero_angle_cmd[0]=0b10000000;                           //WRITE REG 0 (8 LSB of zero angle)
			set_zero_angle_cmd[1]=(uint8_t) zero_point;                 //ZERO ANGLE SET TO CURRENT ANGLE
			pthread_mutex_lock(&lock);
			gpio_result = gpioWrite(cs_angle_sensor_2,0);
			spi_result = spiXfer(handle, set_zero_angle_cmd, inBuf, 2);
			gpio_result = gpioWrite(cs_angle_sensor_2,1);
			pthread_mutex_unlock(&lock);
			usleep(100000);
			pthread_mutex_lock(&lock);
			gpio_result = gpioWrite(cs_angle_sensor_2,0);
			spi_result = spiXfer(handle, read_angle_cmd, inBuf, 2);
			gpio_result = gpioWrite(cs_angle_sensor_2,1);
			pthread_mutex_unlock(&lock);
			usleep(100000);


			//*********PRINT RESULT****************
			//*************************************
			pthread_mutex_lock(&lock);
			gpio_result = gpioWrite(cs_angle_sensor_1,0);
			spi_result = spiXfer(handle, read_angle_cmd, inBuf, 1);
			gpio_result = gpioWrite(cs_angle_sensor_1,1);
			pthread_mutex_unlock(&lock);
			theta1=inBuf[0];

			pthread_mutex_lock(&lock);
			gpio_result = gpioWrite(cs_angle_sensor_2,0);
			spi_result = spiXfer(handle, read_angle_cmd, inBuf, 1);
			gpio_result = gpioWrite(cs_angle_sensor_2,1);
			pthread_mutex_unlock(&lock);
			theta2=inBuf[0];

			printf("After calibration: %d | %d \n",	theta1, theta2 );

			//Shut of motors
			torque_cmd[0]=(uint8_t) 0;
			torque_cmd[1]=(uint8_t) 20;
			torque_cmd[2]=(uint8_t) 20;
			pthread_mutex_lock(&lock);
			gpio_result = gpioWrite(cs_output,0);
			spi_result = spiXfer(handle, torque_cmd, inBuf, 3);
			gpio_result = gpioWrite(cs_output,1);
			pthread_mutex_unlock(&lock);

			//Tell SPI thread to include sensors in measurement loop
			pthread_mutex_lock(&lock);
			spi_mem_shared[0] = 1;
			pthread_mutex_unlock(&lock);
			//Wait for a controller to be selected
			while(1){
				sleep(2);
				update_local_zmq_mem();
				if ( !(controller_select==1) ){
					break;
				}
			}
		}


		void jointspace_ijc_pid(){
			while(1){
				//Check instructions
				update_local_zmq_mem();
				//Exit and stop motors if this is not the correct controller
				if ( !(controller_select == 2) ){
					torque1 = 0;
					torque2 = 0;
					update_shared_spi_mem();
					break;
				}

				//Read sensors
				update_local_spi_mem();
				//Proportional controller
				pid_ijc_js.error1 = *(pid_ijc_js.theta1_setpoint) - theta1;
				torque1 = pid_ijc_js.error1*pid_ijc_js.kp1;
				pid_ijc_js.error2 = *(pid_ijc_js.theta2_setpoint) - theta2;
				torque2 = pid_ijc_js.error2*pid_ijc_js.kp2;
				//Give output to SPI thread
				update_shared_spi_mem();

				//Print status every 1000 cycles
				itr_counter++;
				if ( itr_counter > 1000){
					printf("This is finger %d\n", id);
					printf("theta1: %d | theta1_setpoint: %d | error1: %d | u1: %d \n", theta1 , *(pid_ijc_js.theta1_setpoint), pid_ijc_js.error1, torque1);
					printf("theta2: %d | theta2_setpoint: %d | error2: %d | u2: %d \n", theta2 , *(pid_ijc_js.theta2_setpoint), pid_ijc_js.error2, torque2);
					itr_counter=0;
				}
			}
		}

		void cartesian_ijc_pid(){
			while(1){
				//Check instructions
				update_local_zmq_mem();
				//Exit and stop motors if this is not the correct controller
				if ( !(controller_select == 2) ){
					torque1 = 0;
					torque2 = 0;
					update_shared_spi_mem();
					break;
				}

				//Read sensors
				update_local_spi_mem();
				//Inverse kinematics. Source: http://www.hessmer.org/uploads/RobotArm/Inverse%2520Kinematics%2520for%2520Robot%2520Arm.pdf
				pid_ijc_cs.temp = (pow( *(pid_ijc_cs.x) ,2) + pow( *(pid_ijc_cs.y) ,2) - pow(pid_ijc_cs.l1,2)-pow(pid_ijc_cs.l2,2))/(2*pid_ijc_cs.l1*pid_ijc_cs.l2);
				pid_ijc_cs.theta2_setpoint = atan2( sqrt( 1-pid_ijc_cs.temp ), pid_ijc_cs.temp );
				pid_ijc_cs.k1 = pid_ijc_cs.l1 + pid_ijc_cs.l2*cos(pid_ijc_cs.theta2_setpoint);
				pid_ijc_cs.k2 = pid_ijc_cs.l2*sin(pid_ijc_cs.theta2_setpoint);
				pid_ijc_cs.gamma = atan2(pid_ijc_cs.k2,pid_ijc_cs.k1);
				pid_ijc_cs.theta1_setpoint = atan2( *(pid_ijc_cs.y), *(pid_ijc_cs.x) ) - pid_ijc_cs.gamma;
				//Run controller
				pid_ijc_cs.error1 = pid_ijc_cs.theta1_setpoint - theta1;
				torque1 = pid_ijc_cs.error1*pid_ijc_cs.kp1;
				pid_ijc_cs.error2 = pid_ijc_cs.theta2_setpoint - theta2;
				torque2 = pid_ijc_cs.error2*pid_ijc_cs.kp2;
				//Give output to SPI thread
				update_shared_spi_mem();

				//Print status every 1000 cycles
				itr_counter++;
				if ( itr_counter > 1000){

					printf("This is finger %d\n", id);
					printf("theta1: %d | theta1_setpoint: %d | error1: %d | u1: %d \n", theta1 , pid_ijc_cs.theta1_setpoint, pid_ijc_cs.error1, torque1);
					printf("theta2: %d | theta2_setpoint: %d | error2: %d | u2: %d \n", theta2 , pid_ijc_cs.theta2_setpoint, pid_ijc_cs.error2, torque2);
					itr_counter = 0;
				}
			}
		}

    void* run(){
			std::cout << "i am thread: "<< id << "  >:O" << std::endl;
      update_local_zmq_mem();
			calibration();
			//While finger is instructed to be active
      while( !(controller_select == 0) ){
				//Cycle through controllers.
        jointspace_ijc_pid();
        cartesian_ijc_pid();
      }
			//Tell spi and zmq thread we are finished
			shutdown();
    }

			//A static function is needed to create a separate thread.
			//This function starts the run() function.
		static void *init_finger(void *finger_object){
			std::cout << "i am static bootstrap of thread" << std::endl;
			return ((finger*)finger_object)->run();
		}
};

class zmq_client{

  private:
  //ZMQ
  char* address;
  char* contents;
  void* context;
  void* subscriber;

  //Input data (payload)
  double data1, data2, data3, data4;
  int finger_select;
  int controller_select;

  //Memory shared by controllers and ZMQ_cleint.
  //Rows:     Finger 1-7  (There is only enough GPIO pins for 7 fingers)
  //Coloums:  run_flag, controller_select, data1, data2, data3, data4
  double (*commands)[6];

  //An array of pointers to the functions that starts each finger
  //void* (* finger_run [7])(void *);
  finger* finger_ptrs[7];
  //Amount of fingers in use
  int finger_count;
  //std::string input_string;
//  std::string stringtrash;
//  zmq::message_t update;
  public:

    zmq_client(double shared_zmq_memory[7][6], finger* fingers[7]){
			commands = shared_zmq_memory;
      //ZMQ setup
      context = zmq_ctx_new ();
      subscriber = zmq_socket (context, ZMQ_SUB);
      zmq_connect (subscriber, "tcp://169.254.27.157:5563");
      zmq_setsockopt (subscriber, ZMQ_SUBSCRIBE, "B", 1);


      //Clear the shared memory that will be used

      //Load pointers to start functions
      for (int i = 0; i < 7; i++){
        finger_ptrs[i] = fingers[i];
      }
    }

    void* run(){
      while(1){
        address = s_recv (subscriber);  //  Read envelope with address
        //contents = s_recv (subscriber); //  Read message contents
        std::string input_string = s_recv (subscriber); //  Read message contents
        //subscriber.recv(&update);
        //std::stringstream string_stream(static_cast<char*>(input_string);
        std::stringstream string_stream;
        string_stream << input_string;
        string_stream >> finger_select >> controller_select >> data1 >> data2 >> data3 >> data4;
				//sscanf(input_string, "%d %d %f %f %f %f",&finger_select , &controller_select , &data1, &data2, &data3, &data4);
      //  std::cout << input_string << std::endl;
        std::cout << (int)finger_select << " " << (int)controller_select<<" " << (double)data1 << " "<< (double)data2 << " " << (double)data3 <<" "<< (double)data4 <<std::endl;
        //finger_select = (uint8_t) contents[0];
				        //If a viable finger is selected (finger 0-4)
        if ( ( 0 < finger_select) && (finger_select <= 7) ){
          //Read and unload data to shared memory

					//std::cout << (int)finger_select << " " << (int)controller_select<<" " << (int)data1 << " "<< (int)data2 << " " << (int)data3 <<" "<< (int)data4 <<std::endl;
					std::cout << "Putting commands in shared memory" << std::endl;
          pthread_mutex_lock(&lock);
          commands[finger_select][1] = controller_select;
          commands[finger_select][2] = data1;
          commands[finger_select][3] = data2;
          commands[finger_select][4] = data3;
          commands[finger_select][5] = data4;

					std::cout << "it worked :O" << std::endl;
					std::cout <<" Here is the runflag: "<< commands[finger_select][0] <<std::endl;
          //If the finger is not running, and the new command is not to stop
          if ( (commands[finger_select][0] == 0) && !(controller_select == 0b00000000) ){
            //Set a flag in shared memory showing that the finger thread is running
            commands[finger_select][0] = 1;
            //Start a the finger on a new thread.
											std::cout << "Attempting to create thread"<< std::endl;
            pthread_create(&(tid[2+finger_select]), NULL, &finger::init_finger, finger_ptrs[finger_select]);
            //Note that the finger thread will terminate on its own
            //and set the run_flag low when controller_select = 0.
          }
          pthread_mutex_unlock(&lock);
						std::cout << "after creating thread" << std::endl;
        }
      }
    };

		static void* init_zmq(void* zmq_object){
			return ((zmq_client*)zmq_object)->run();
		}
};

class spi{
  private:
		double (*shared_mem)[7];
		double local_mem[7][7];

    unsigned frequency;
    unsigned spi_channel;
    int sclk;
    int mosi;
    int miso;

    int cs_f1_sens1, cs_f1_sens2, cs_f1_esp32;
    int cs_f2_sens1, cs_f2_sens2, cs_f2_esp32;
    int cs_f3_sens1, cs_f3_sens2, cs_f3_esp32;
    int cs_f4_sens1, cs_f4_sens2, cs_f4_esp32;
    int cs_f5_sens1, cs_f5_sens2, cs_f5_esp32;
    int cs_f6_sens1, cs_f6_sens2, cs_f6_esp32;
    int cs_f7_sens1, cs_f7_sens2, cs_f7_esp32;
    int (*cs_arr)[3];

    //SHARED MEMORY
		//Rows: finger 0-6
		//Coloums: spi_flag angle1, angle2, velocity1, velocity2, output1, output2
		//int16_t shared_spi[7][7];
		//uint16_t local_spi[7][7];


    char inBuf[4];
    char outBuf[4];
    char read_command_8;
    char read_command_16[2];
    uint16_t temp;

    uint16_t temp_angle1;
    uint16_t temp_angle2;
    double temp_output1;
    double temp_output2;

		int run_flag[5];

		int time0;
		int time1;
		int step;

		int gpio_result;
		int spi_result;
		int spi_handle;

  public:
    spi(double shared_spi_memory[7][7], int chip_selects[7][3]){

      if (gpioInitialise() < 0)
      {
         //printf(stderr, "pigpio initialisation failed.\n");
         std::cout << "pigpio initialisation failed" << std::endl;
      }

			shared_mem = shared_spi_memory;
			cs_arr = chip_selects;


      //SPI frequency
      frequency = 15000000;
			spi_channel = 0;
			spi_handle = spiOpen(spi_channel, frequency, 0);
			std::cout << "THIS IS MY MF HANDLE BOIIIS" << spi_handle << std::endl;
      //SPI channel. Using this channel means that GPIO 8 is used as chip select.
      //It will however not be connected to anything, and only used because the
      //SPI driver requires a channel to be chosen. Since there are only two channels,
      //and we need more, we manualy activate other GPIO pins

      //Common pins. All SPI devies are connected to these.
      sclk = 11;
      mosi = 10;
      miso = 9;

      //The chip select that is actually connected to devices
      //Note that this is GPIO number and not pin number

      //Data is not transmitted when cs is high. Therefore; set all to high
      for (int i=0; i <= 6; i++){
        for (int j=0; j <=3; j++){
          gpio_result = gpioWrite(cs_arr[i][j], 1);
          std::cout <<"IS THIS THE PLACE?"<<std::endl;
        }
      }

      read_command_8 = 0b00000000;
      read_command_16[0] = 0b00000000;
      read_command_16[1] = 0b00000000;

    }

    uint8_t read_angle_8(int &cs){
      outBuf[0] = read_command_8;
			pthread_mutex_lock(&lock);
      gpio_result = gpioWrite(cs,0);
      spi_result = spiXfer(spi_handle, outBuf, inBuf, 1);
      gpio_result = gpioWrite(cs,1);
			pthread_mutex_unlock(&lock);
      return inBuf[0];
    }
    uint16_t read_angle_16(int &cs){
      outBuf[0] = read_command_16[0];
      outBuf[1] = read_command_16[1];
			pthread_mutex_lock(&lock);
      gpio_result = gpioWrite(cs,0);
      spi_result = spiXfer(spi_handle, outBuf, inBuf, 2);
      gpio_result = gpioWrite(cs,1);
			pthread_mutex_unlock(&lock);
      temp = inBuf[0] << 8;
      temp = temp + inBuf[1];
      return temp;
    }
		void write_output_8(int &cs, double &output1, double &output2){
			outBuf[0] = 0;
			if ( output1 < 0 ){
				output1 = output1*(-1);
				outBuf[0] = 0b00000001;
			}
			if (output1 > 255){
				output1 = 255;
			}
			if ( output2 < 0 ){
				output2 = output2*(-1);
				outBuf[0] += 0b00000010;
			}
			if (output2 > 255){
				output2 = 255;
			}
			outBuf[1] =  (uint8_t) output1;
			outBuf[2] =  (uint8_t) output2;

			pthread_mutex_lock(&lock);
			gpio_result = gpioWrite(cs,0);
			spi_result = spiXfer(spi_handle, outBuf, inBuf, 3);
			gpio_result = gpioWrite(cs,1);
			pthread_mutex_unlock(&lock);
		}

		~spi(){
			if (spiClose(spi_handle) < 0){
				printf("spi destroctor\n");
				printf("Bad handle");
			}
			gpioTerminate();
      printf("we destructed");
		}


		int get_cs_and_handle(){
			return spi_handle;
		}

    void* run(){
			while(1){

        std::cout << "This is the spi loop" << std::endl;
				time0=micros();
				//Load info about active fingers
				pthread_mutex_lock(&lock);
				for (int i=0; i<7; i++){
						local_mem[i][0] = shared_mem[i][0];
					}
				pthread_mutex_unlock(&lock);

				for (int i=0; i<7; i++){
					//If fingers are active
					if (local_mem[i][0]){
						//Read angle from SPI
						local_mem[i][1] = read_angle_16(cs_arr[i][1]);
						local_mem[i][2] = read_angle_16(cs_arr[i][2]);

						//Calculate speed
						//************** NOT DONE *********************

						pthread_mutex_lock(&lock);
						//Update shared memory
						shared_mem[i][1] = local_mem[i][1];
						shared_mem[i][2] = local_mem[i][2];
						shared_mem[i][3] = local_mem[i][3];
						shared_mem[i][4] = local_mem[i][4];

						//Read output from shared memory
						local_mem[i][5] = shared_mem[i][5];
						local_mem[i][6] = shared_mem[i][6];
						pthread_mutex_unlock(&lock);

						//Send output to SPI
						write_output_8(cs_arr[i][3],local_mem[i][5],local_mem[i][6]);
					}
				}
				//WAIT
				time1=micros();
				step=time1-time0;
				while(step<1000){
					time1=micros();
					step=time1-time0;
					usleep(250);
					//std::cout << "we waited" << std::endl;
				}
			}
		}

		static void* init_spi(void *spi_object){
			return ((spi* )spi_object)->run();
		}
};

main(){
  //Initiate finger objects. The arguments is the identity of the finger.
  //The identity corresponds to specific SPI pins. Choose a value between 0-6.
  //Additional fingers can be added (max 7 with the amount of GPIO pins on a RaspberryPi).

	int cs_f1_sens1 = 2;
	int cs_f1_sens2 = 3;
	int cs_f1_esp32 = 4;
	int cs_f2_sens1 = 5;
	int cs_f2_sens2 = 6;
	int cs_f2_esp32 = 7;
	int cs_f3_sens1 = 12;
	int cs_f3_sens2 = 13;
	int cs_f3_esp32 = 14;
	int cs_f4_sens1 = 15;
	int cs_f4_sens2 = 16;
	int cs_f4_esp32 = 17;
	int cs_f5_sens1 = 18;
	int cs_f5_sens2 = 19;
	int cs_f5_esp32 = 20;
	int cs_f6_sens1 = 21;
	int cs_f6_sens2 = 22;
	int cs_f6_esp32 = 23;
	int cs_f7_sens1 = 24;
	int cs_f7_sens2 = 25;
	int cs_f7_esp32 = 26;
	//Cs is packed into array so that it can be accessed by ID
	int cs_arr[7][3] = 	{	 {cs_f1_sens1, cs_f1_sens2, cs_f1_esp32}
								 	 			,{cs_f2_sens1, cs_f2_sens2, cs_f2_esp32}
									 			,{cs_f3_sens1, cs_f3_sens2, cs_f3_esp32}
									 			,{cs_f4_sens1, cs_f4_sens2, cs_f4_esp32}
									 			,{cs_f5_sens1, cs_f5_sens2, cs_f5_esp32}
									 			,{cs_f6_sens1, cs_f6_sens2, cs_f6_esp32}
									 			,{cs_f7_sens1, cs_f7_sens2, cs_f7_esp32}};

	//SPI and ZMQ threads share memory with fingers.
	double shared_spi_memory[7][7];
	double shared_zmq_memory[7][6];

	std::fill( shared_spi_memory[0], shared_spi_memory[0] + 7*7, 0);
	std::fill( shared_zmq_memory[0], shared_zmq_memory[0] + 7*6, 0);

  spi spi_controller(shared_spi_memory,cs_arr);

	//Creating finger objects and hooking them up to shared memory shared by zmq and spi threads
  finger finger1(1,&shared_spi_memory[0][0], &shared_zmq_memory[0][0], &cs_arr[0][0]);
  finger finger2(2,&shared_spi_memory[1][0], &shared_zmq_memory[1][0], &cs_arr[1][0]);
  finger finger3(3,&shared_spi_memory[2][0], &shared_zmq_memory[2][0], &cs_arr[2][0]);
	finger finger4(4,&shared_spi_memory[3][0], &shared_zmq_memory[3][0], &cs_arr[3][0]);
	finger finger5(5,&shared_spi_memory[4][0], &shared_zmq_memory[4][0], &cs_arr[4][0]);
	finger finger6(6,&shared_spi_memory[5][0], &shared_zmq_memory[5][0], &cs_arr[5][0]);
	finger finger7(7,&shared_spi_memory[6][0], &shared_zmq_memory[6][0], &cs_arr[5][0]);

  pthread_create(&(tid[0]), NULL, &spi::init_spi, &spi_controller);

  //Create an array of function pointers
  //Fill the array with the address of the function that starts each finger
  //Create a ZMQ client. With number of fingers and the function pointer array as argument
  //Run the ZMQ client on separate thread.
/*
  void* (* finger_run_fct_ptr [7])(void *);
  finger_run_fct_ptr[0] = finger1.run;
  finger_run_fct_ptr[1] = finger2.run;
	finger_run_fct_ptr[2] = finger3.run;
	finger_run_fct_ptr[3] = finger4.run;
	finger_run_fct_ptr[4] = finger5.run;
	finger_run_fct_ptr[5] = finger6.run;
	finger_run_fct_ptr[6] = finger7.run;
	*/
	finger* finger_ptr[7];
	finger_ptr[0] = &finger1;
	finger_ptr[1] = &finger2;
	finger_ptr[2] = &finger3;
	finger_ptr[3] = &finger4;
	finger_ptr[4] = &finger5;
	finger_ptr[5] = &finger6;
	finger_ptr[6] = &finger7;

  zmq_client zmq(shared_zmq_memory, finger_ptr);
  pthread_create(&(tid[1]), NULL, &zmq_client::init_zmq, &zmq);


  pthread_join(tid[0], NULL);
  pthread_join(tid[1], NULL);
}
