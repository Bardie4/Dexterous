// gcc -Wall -pthread -o bbSPIx_test bbSPIx_test.c -lpigpio
// sudo ./bbSPIx_test

//  Hello World client


#include "zhelpers.h"
#include <stdio.h>
#include <pigpio.h>
#include <unistd.h>
//#include <bitset>
#include "pthread.h"

#define link1 5
#define link2 6
#define esp 13
#define MISO 19
#define MOSI 26
#define SCLK 21



typedef struct read_zmq_bundle {
   char* address;
   char* contents;
   void* context;
   void* subscriber;
   int link1_angle;
   int link2_angle;
}read_zmq_bundle;


pthread_t tid[2];
pthread_mutex_t lock;

void* pid_(void* zmq_read_input){
  pthread_mutex_lock(&lock);
  read_zmq_bundle* zmq_read = (read_zmq_bundle*)zmq_read_input;
  pthread_mutex_unlock(&lock);
  while(1){

  pthread_mutex_lock(&lock);

  printf("while out mutex");
  /*
  //Read angle
  char read_angle_cmd[]= {0b00000000, 0b00000000};
  unsigned char inBuf[4];
  uint8_t theta1;
  uint8_t theta2;
  int error1;
  int error2;
  bbSPIXfer(link1, read_angle_cmd, (char *)inBuf, 1); // > DAC
  theta1=inBuf[0];
  error1= (int short)( ((uint8_t) zmq_read->link1_angle)-inBuf[0]);
  bbSPIXfer(link2, read_angle_cmd, (char *)inBuf, 1); // > DAC
  theta2=inBuf[0];
  error1= (int short)( ((uint8_t) zmq_read->link2_angle)-inBuf[0]);
  //PID
  */


  printf("I am now reading from memory modified on another thread: %d | %d \n",zmq_read->link1_angle, zmq_read->link2_angle);
  //printf("erro1: %d | error2: %d \n",error1,error2);
//Report angle (For testing)
//cout_itr++;

  pthread_mutex_unlock(&lock);
  usleep(1000000);
  }
}

//Output

// count = bbSPIXfer(esp, torque_cmd, (char *)inBuf, 4);
//   cout << "ZMQ: "<<endl;
//  Read envelope with address

//   cout << "[" << address << "] " << contents << std::endl;

void* read_reference_angle(void* zmq_read_input){
  pthread_mutex_lock(&lock);
  read_zmq_bundle* zmq_read = (read_zmq_bundle*) zmq_read_input;
  pthread_mutex_unlock(&lock);
  while (1) {
      pthread_mutex_lock(&lock);
      printf("while read mutex");
      //  Read envelope with address
      zmq_read->address = s_recv (zmq_read->subscriber);
      //  Read message contents
      zmq_read->contents = s_recv (zmq_read->subscriber);
      //printf("%s\n", contents);
      sscanf(zmq_read->contents, "%d %d", &(zmq_read->link1_angle), &(zmq_read->link2_angle));
      //printf("| %s %s\n", garbage1,garbage2);
      //sscanf(contents, "%lf[^ ]%lf[^\n]", &link1_angle, &link2_angle);
      printf("%d %d\n", zmq_read->link1_angle, zmq_read->link2_angle);
      //printf("%s\n", contents);
      free (zmq_read->address);
      free (zmq_read->contents);

      printf("while read mutex end");
      pthread_mutex_unlock(&lock);
      usleep(500000);
    }
}


int main()
{
  read_zmq_bundle zmq_read;
  //  Prepare our context and subscriber
  zmq_read.context = zmq_ctx_new ();
  zmq_read.subscriber = zmq_socket (zmq_read.context, ZMQ_SUB);

  //void *context = zmq_ctx_new ();
  //void *subscriber = zmq_socket (context, ZMQ_SUB);
  //zmq_connect (subscriber, "tcp://10.218.130.229:5563");
  zmq_connect (zmq_read.subscriber, "tcp://169.254.27.157:5563");
  zmq_setsockopt (zmq_read.subscriber, ZMQ_SUBSCRIBE, "B", 1);
  printf("Before threading");
  pthread_create(&(tid[0]), NULL, &read_reference_angle, &zmq_read);
  usleep(1000000);
  pthread_create(&(tid[1]), NULL, &pid_, &zmq_read);
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

   //SPI INITATION
   SPI_init1 = bbSPIOpen(link1, MISO, MOSI, SCLK, 250000, 3);
   SPI_init2 = bbSPIOpen(link2, MISO, MOSI, SCLK, 250000, 3);
   SPI_init3 = bbSPIOpen(esp, MISO, MOSI, SCLK, 250000, 3);
   //cout << "Initiation of spi1: " << SPI_init1 << endl;
   //cout << "Initiation of spi2: " << SPI_init2 << endl;
   //cout << "Initiation of spi3: " << SPI_init3 << endl;


   //Report start angle
   count = bbSPIXfer(link1, read_angle_cmd, (char *)inBuf, 1);
   theta1=inBuf[0];
   count = bbSPIXfer(link2, read_angle_cmd, (char *)inBuf, 1);
   theta2=inBuf[0];
   //cout  << "link1 angle: " << unsigned(theta1) <<"  link2 angle: " << unsigned(theta2) << endl;

   //Start by focing motors to start position
   torque_cmd[0]=(uint8_t) 0;
   torque_cmd[1]=(uint8_t) 20;
   torque_cmd[2]=(uint8_t) 0;
   torque_cmd[3]=(uint8_t) 20;

   sleep(3);

   //Setting zero_angle at start position
   //The measured angle in end position should be zero to avoid crossing from 0->255, as this will mess with the PID.
   //Any previous zero angle setting is removed before the angle is measured. This measured angle is set as the new zero angle.
   //A delay followed by 16 zeros is required after each write to sensor register.
   //The register value should be the compliment of the wanted zero-angle
   //SENSOR 1
   set_zero_angle_cmd[0]=0b10000001; //WRITE REG 1 (8 MSB of zero angle)
   set_zero_angle_cmd[1]=0b00000000; //ZERO-ANGLE SET TO 0
   count = bbSPIXfer(link1, set_zero_angle_cmd, (char *)inBuf, 2);
   usleep(50000);
   count = bbSPIXfer(link1, read_angle_cmd, (char *)inBuf, 2);
  // cout  << "Register value: " << bitset<8>(inBuf[0]) <<"| zeros " << bitset<8>(inBuf[1]) << endl;
   usleep(50000);
   set_zero_angle_cmd[0]=0b10000000; //WRITE REG 0 (8 LSB of zero angle)
   set_zero_angle_cmd[1]=0b00000000; //ZERO-ANGLE SET TO 0
   count = bbSPIXfer(link1, set_zero_angle_cmd, (char *)inBuf, 2);
   usleep(50000);
   count = bbSPIXfer(link1, read_angle_cmd, (char *)inBuf, 2);
   //cout  << "Register value: " <<  bitset<8>(inBuf[0]) <<"| zeros " << bitset<8>(inBuf[1]) << endl;
   usleep(50000);

   count = bbSPIXfer(link1, read_angle_cmd, (char *)inBuf, 2); //MEASURE CURRENT ANGLE
   zero_point = (inBuf[0] << 8);                               //COMBINE 8 bit values to 16 bit
   zero_point = zero_point + inBuf[1];
   //cout << "zero_point_16: " << zero_point <<endl;
   //cout << "zero_point_8: " << unsigned((zero_point >> 8)) << endl;
   zero_point = (uint16_t) (0b10000000000000000-zero_point);   //CALCULATE COMPLIMENT (Formula 4 in Datasheet:  MagAlpha MA302  12-Bit, Digital, Contactless Angle Sensor with ABZ & UVW Incremental Outputs )

   //cout << "zero_point_compliment_16: " << zero_point << endl;
  // cout << "zero_point__compliment_bit: "<< bitset<16>(zero_point) << endl;
   set_zero_angle_cmd[0]=0b10000001;
   set_zero_angle_cmd[1]=(uint8_t) (zero_point >> 8);          //8 MSB of Compliment of new zero angle
   //cout << bitset<8>(set_zero_angle_cmd[1]) << endl;
   count = bbSPIXfer(link1, set_zero_angle_cmd, (char *)inBuf, 2);
   usleep(50000);
   count = bbSPIXfer(link1, read_angle_cmd, (char *)inBuf, 2);
  // cout  << "Register value: " << bitset<8>(inBuf[0]) <<"| zeros " << bitset<8>(inBuf[1]) << endl;
   usleep(50000);
   set_zero_angle_cmd[0]=0b10000000;
   set_zero_angle_cmd[1]=(uint8_t) zero_point;                 //8 LSB of Compliment of new zero angle
   count = bbSPIXfer(link1, set_zero_angle_cmd, (char *)inBuf, 2);
   usleep(50000);
   count = bbSPIXfer(link1, read_angle_cmd, (char *)inBuf, 2);
//   cout  << "Register value: " <<  bitset<8>(inBuf[0]) <<"| zeros " << bitset<8>(inBuf[1]) << endl;
   usleep(50000);

   //SENSOR 2
   set_zero_angle_cmd[0]=0b10000001;   //WRITE REG 1 (8 MSB of zero angle)
   set_zero_angle_cmd[1]=0b00000000;   //RESET ZERO ANGLE
   count = bbSPIXfer(link2, set_zero_angle_cmd, (char *)inBuf, 2);
   usleep(50000);
   count = bbSPIXfer(link2, read_angle_cmd, (char *)inBuf, 2);
  // cout  << "Register value: " << bitset<8>(inBuf[0]) <<"| zeros " << bitset<8>(inBuf[1]) << endl;
   usleep(50000);
   set_zero_angle_cmd[0]=0b10000000;   //WRITE REG 0 (8 LSB of zero angle)
   set_zero_angle_cmd[1]=0b00000000;   //RESET ZERO ANGLE
   count = bbSPIXfer(link2, set_zero_angle_cmd, (char *)inBuf, 2);
   usleep(50000);
   count = bbSPIXfer(link2, read_angle_cmd, (char *)inBuf, 2);
  // cout  << "Register value: " <<  bitset<8>(inBuf[0]) <<"| zeros " << bitset<8>(inBuf[1]) << endl;
   usleep(50000);

   count = bbSPIXfer(link2, read_angle_cmd, (char *)inBuf, 2); // MEASURE ZERO ANGLE
   zero_point = (inBuf[0] << 8);
   zero_point = zero_point + inBuf[1];
  // cout << "zero_point_16: " << zero_point <<endl;
  // cout << "zero_point_8: " << unsigned((zero_point >> 8)) << endl;
   zero_point = (uint16_t) (0b10000000000000000-zero_point);
  // cout << "zero_point_compliment_16: " << zero_point << endl;
  // cout << "zero_point__compliment_bit: "<< bitset<16>(zero_point) << endl;
   set_zero_angle_cmd[0]=0b10000001;                           //WRITE REG 1 (8 MSB of zero angle)
   set_zero_angle_cmd[1]=(uint8_t) (zero_point >> 8);          //ZERO ANGLE SET TO CURRENT ANGLE
   //cout << bitset<8>(set_zero_angle_cmd[1]) << endl;
   count = bbSPIXfer(link2, set_zero_angle_cmd, (char *)inBuf, 2);
   usleep(50000);
   count = bbSPIXfer(link2, read_angle_cmd, (char *)inBuf, 2);
//   cout  << "Register value: " << bitset<8>(inBuf[0]) <<"| zeros " << bitset<8>(inBuf[1]) << endl;
   usleep(50000);
   set_zero_angle_cmd[0]=0b10000000;                           //WRITE REG 0 (8 LSB of zero angle)
   set_zero_angle_cmd[1]=(uint8_t) zero_point;                 //ZERO ANGLE SET TO CURRENT ANGLE
   count = bbSPIXfer(link2, set_zero_angle_cmd, (char *)inBuf, 2);
   usleep(50000);
   count = bbSPIXfer(link2, read_angle_cmd, (char *)inBuf, 2);
  // cout  << "Register value: " <<  bitset<8>(inBuf[0]) <<"| zeros " << bitset<8>(inBuf[1]) << endl;
   usleep(50000);


   //Report new angle with modified zero angle:
   count = bbSPIXfer(link1, read_angle_cmd, (char *)inBuf, 1);
   theta1=inBuf[0];
   count = bbSPIXfer(link2, read_angle_cmd, (char *)inBuf, 1);
   theta2=inBuf[0];
//   cout  << "New link1 angle: " << unsigned(theta1) <<"New link2 angle " << unsigned(theta2) << endl;

   usleep(50000);
   pthread_create(&(tid[0]), NULL, &read_reference_angle, &zmq_read);
   usleep(1000000);
   pthread_create(&(tid[1]), NULL, &pid_, &zmq_read);
   if (pthread_mutex_init(&lock, NULL) != 0)
   {
       printf("\n mutex init failed\n");
       return 1;
   }*//*
   while (1)
   {
      //Read angle
      count = bbSPIXfer(link1, read_angle_cmd, (char *)inBuf, 1); // > DAC
      theta1=inBuf[0];
      //count = bbSPIXfer(link2, read_angle_cmd, (char *)inBuf, 1); // > DAC
      //theta2=inBuf[0];

      //PID
      error1= (int short) zmq_read.link1_angle-theta1;
      error2= (int short) zmq_read.link2_angle-theta2;

      u1=kp1*error1;
      u2=kp2*error2;


      printf("I am now reading from memory modified on another thread: %d | %d \n",zmq_read.link1_angle, zmq_read.link2_angle);

	 //Report angle (For testing)
	  cout_itr++;
	  if (cout_itr > 1000)
	  {
	//	  cout << "link1 angle: " << unsigned(theta1) << " link1 error: " << error1 << " u1: " << u1 << "| link2 angle: " << unsigned(theta2) << " link2 error: " << error2 << " u2: " << u2 << endl;
	//	  cout << "u1 bit string: "<< bitset<16>(u1) << "  " << bitset<8>(torque_cmd[0]) << bitset<8>(torque_cmd[1]) << " | u1 bit string: " << bitset<16>(u1) << "  " << bitset<8>(torque_cmd[2]) << bitset<8>(torque_cmd[3]) << endl;
		  cout_itr = 0;
	  }

	  //Output

	 // count = bbSPIXfer(esp, torque_cmd, (char *)inBuf, 4);
//   cout << "ZMQ: "<<endl;
   //  Read envelope with address

//   cout << "[" << address << "] " << contents << std::endl;
}*/
   //mq_close (zmq_read.subscriber);
   //zmq_ctx_destroy (zmq_read.context);
   //return 0;

   /*
   for (i=0; i<256; i++)
   {
      cmd1[1] = i;

      count = bbSPIXfer(CE0, cmd1, (char *)inBuf, 2); // > DAC

      if (count == 2)
      {
         count = bbSPIXfer(CE0, cmd2, (char *)inBuf, 2); // < DAC

         if (count == 2)
         {
            set_val = inBuf[1];
            count = bbSPIXfer(CE1, cmd3, (char *)inBuf, 3); // < ADC

            if (count == 3)
            {
               read_val = ((inBuf[1]&3)<<8) | inBuf[2];
               printf("%d %d\n", set_val, read_val);
            }
         }
      }
   }
   *//*
   bbSPIClose(link1);
   bbSPIClose(link2);
   bbSPIClose(esp);

   gpioTerminate();

   return 0;
   */
}
