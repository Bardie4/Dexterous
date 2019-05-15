#include <pigpio.h>
#include <iostream>
#include <atomic>
#include <signal.h>
#include <string.h>
#include <unistd.h>

using namespace std;

//g++ -Wall -pthread -o test test.cpp -lpigpio -lrt

int main(int argc, char** argv)
{
    gpioInitialise();

    unsigned i2cBus = 1;
    unsigned i2cAddr = 0x28; // 0x28 ++
    unsigned i2cFlags = 0;

    int i2cHandle = i2cOpen(i2cBus, i2cAddr, i2cFlags);
    char buf[3] = {0x01, 0x02, 0x03};

    int i2cHandle2 = i2cOpen(i2cBus, 0x29, i2cFlags);
    char buf2[3] = {0x04, 0x05, 0x06};

    i2cWriteDevice(i2cHandle, buf, sizeof(buf));
    cout << i2cHandle << endl;
    i2cWriteDevice(i2cHandle2, buf2, sizeof(buf2));
    cout << i2cHandle2 << endl;

    i2cClose(i2cHandle);
    i2cClose(i2cHandle2);
    gpioTerminate();
    return 0;
}
