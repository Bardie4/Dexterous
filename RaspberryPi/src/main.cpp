#include <iostream>
#include <errno.h>
#include <wiringPiSPI.h>
#include <unistd.h>
using namespace std;



static const int SENSOR_CHANNEL = 0;
static const int ESP_CHANNEL = 1;
static const int SPI_SCLK_FREQUENCY=100000;

int main()
{           
    int sensor;
    sensor = wiringPiSPISetupMode (SENSOR_CHANNEL, SPI_SCLK_FREQUENCY, 0);
    cout << "Init result: " << sensor << endl;
    //Angle variable
    double theta;
    //Buffer
    unsigned char spi_buffer[1];
    int length=1;
    while (1){
       spi_buffer[0]=0x00;
       theta=wiringPiSPIDataRW (SENSOR_CHANNEL, spi_buffer, length);
       cout << "Angle: " << buffer[0] << endl;
    }
}