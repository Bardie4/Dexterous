// gcc -Wall -pthread -o bbSPIx_test bbSPIx_test.c -lpigpio
// sudo ./bbSPIx_test

#include <stdio.h>
#include <iostream>
#include <pigpio.h>
#include <unistd.h>
#include <bitset>

#define link1 5
#define link2 6
#define esp 13
#define MISO 19
#define MOSI 26
#define SCLK 21


using namespace std;

int main()
{
   int count, set_val, read_val, x, SPI_init1, SPI_init2, SPI_init3, cout_itr=1;
   unsigned char inBuf[4];
   char read_angle_cmd[]= {0b00000000};
   char set_zero_angle_cmd[2];
   char torque_cmd[4];

   //PID
   double kp1=0.1;
   double kp2=0.1;
   double max_torque1=0.3;    //unused
   double max_torque2=0.15;   //unused
   

   uint8_t theta1;
   uint8_t theta2;
   uint8_t theta1_0;    //Angle bias link 1
   uint8_t theta2_0;    //Angle bias link 2
   uint8_t setpoint1;
   uint8_t setpoint2;
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
   SPI_init1 = bbSPIOpen(link1, MISO, MOSI, SCLK, 10000, 3);
   SPI_init2 = bbSPIOpen(link2, MISO, MOSI, SCLK, 10000, 3);
   SPI_init3 = bbSPIOpen(esp, MISO, MOSI, SCLK, 10000, 3);
   cout << "Initiation of spi1: " << SPI_init1 << endl;
   cout << "Initiation of spi2: " << SPI_init2 << endl;
   cout << "Initiation of spi3: " << SPI_init3 << endl;

   
   //Report start angle
   count = bbSPIXfer(link1, read_angle_cmd, (char *)inBuf, 1); // > DAC
   theta1=inBuf[0];
   count = bbSPIXfer(link2, read_angle_cmd, (char *)inBuf, 1); // > DAC
   theta2=inBuf[0];
   cout  << "link1 angle: " << unsigned(theta1) <<"  link2 angle: " << unsigned(theta2) << endl; 

   //Start by focing motors to start position
   torque_cmd[0]=(uint8_t) 0;
   torque_cmd[1]=(uint8_t) 20;
   torque_cmd[2]=(uint8_t) 0;
   torque_cmd[3]=(uint8_t) 20;

   sleep(3);

   //Setting zero_angle at start position
   count = bbSPIXfer(link1, read_angle_cmd, (char *)inBuf, 1); // > DAC
   set_zero_angle_cmd[0]=0b10000001;
   set_zero_angle_cmd[1]=0b11110001;
   count = bbSPIXfer(link1, set_zero_angle_cmd, (char *)inBuf, 2); // > DAC
   set_zero_angle_cmd[0]=0b10000000;
   set_zero_angle_cmd[1]=0b11000111;
   count = bbSPIXfer(link1, set_zero_angle_cmd, (char *)inBuf, 2); // > DAC

   //count = bbSPIXfer(link2, read_angle_cmd, (char *)inBuf, 1); // > DAC
   set_zero_angle_cmd[1]=0b10000001;
   set_zero_angle_cmd[0]=255-inBuf[0];
   //count = bbSPIXfer(link2, set_zero_angle_cmd, (char *)inBuf, 2); // > DAC
   //set_zero_angle_cmd[0]=0b10000000;
   //set_zero_angle_cmd[1]=0b00000000;
   //count = bbSPIXfer(link2, set_zero_angle_cmd, (char *)inBuf, 2); // > DAC

   //Report startangle
   count = bbSPIXfer(link1, read_angle_cmd, (char *)inBuf, 1); // > DAC
   theta1=inBuf[0];
   count = bbSPIXfer(link2, read_angle_cmd, (char *)inBuf, 1); // > DAC
   theta2=inBuf[0];
   cout  << "New link1 angle: " << unsigned(theta1) <<"New link2 angle " << unsigned(theta2) << endl; 
   
   while (1)
   {
      //Read angle
      count = bbSPIXfer(link1, read_angle_cmd, (char *)inBuf, 1); // > DAC
      theta1=inBuf[0];
      //count = bbSPIXfer(link2, read_angle_cmd, (char *)inBuf, 1); // > DAC
      //theta2=inBuf[0];
      
      //PID
      error1= (int short) setpoint1-theta1;
      error2= (int short) setpoint2-theta2;

      u1=kp1*error1;
      u2=kp2*error2;

       


	 //Report angle (For testing)
	  cout_itr++;
	  if (cout_itr > 1000)
	  {
		  cout << "link1 angle: " << unsigned(theta1) << " link1 error: " << error1 << " u1: " << u1 << "| link2 angle: " << unsigned(theta2) << " link2 error: " << error2 << " u2: " << u2 << endl;
		  cout << "u1 bit string: "<< bitset<16>(u1) << "  " << bitset<8>(torque_cmd[0]) << bitset<8>(torque_cmd[1]) << " | u1 bit string: " << bitset<16>(u1) << "  " << bitset<8>(torque_cmd[2]) << bitset<8>(torque_cmd[3]) << endl;
		  cout_itr = 0;
	  }

	  //Output
     
	 // count = bbSPIXfer(esp, torque_cmd, (char *)inBuf, 4);
     
   }
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
   */
   bbSPIClose(link1);
   bbSPIClose(link2);
   bbSPIClose(esp);

   gpioTerminate();

   return 0;
}