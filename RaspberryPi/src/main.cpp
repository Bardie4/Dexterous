// gcc -Wall -pthread -o bbSPIx_test bbSPIx_test.c -lpigpio
// sudo ./bbSPIx_test

#include <stdio.h>
#include <iostream>
#include <pigpio.h>
#include <unistd.h>

#define CE0 5
#define CE1 6
#define MISO 13
#define MOSI 19
#define SCLK 12


using namespace std;

int main()
{
   int i, count, set_val, read_val, x, SPI_init;
   unsigned char inBuf[2];
   char cmd1[]= {0,0};

   if (gpioInitialise() < 0)
   {
      fprintf(stderr, "pigpio initialisation failed.\n");
      return 1;
   }

   SPI_init = bbSPIOpen(CE0, MISO, MOSI, SCLK, 100, 2); // MCP4251 DAC
   //bbSPIOpen(CE1, MISO, MOSI, SCLK, 20000, 3); // MCP3008 ADC
   cout << "Initiation of spi: " << SPI_init << endl;

   while (x < 30)
   {
        count = bbSPIXfer(CE0, cmd1, (char *)inBuf, 1); // > DAC
        cout  << unsigned(inBuf[0]) <<" __ " << unsigned(inBuf[1]) << " __ " << count <<endl; 
        sleep(unsigned(1));
        x++;
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
   bbSPIClose(CE0);

   gpioTerminate();

   return 0;
}