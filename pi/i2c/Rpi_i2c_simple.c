
#include <stdio.h>
#include <stdlib.h>
#include <pigpio.h>

int main(int argc, char *argv[])
{
   if (gpioInitialise()<0) exit(1);

   gpioSetMode(23, PI_OUTPUT);
   
   gpioWrite(0x28, 1);

   int handle = i2cOpen(1, 0x28, 0);
   i2cWriteQuick(handle, 0b1);

   gpioTerminate();
}
