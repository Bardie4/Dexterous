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
    uint8_t theta;
    double idk;
    //Buffer
    unsigned char spi_buffer[1];
    int length=1;
    while (1){
       spi_buffer[0]=0x00;
       idk=wiringPiSPIDataRW (SENSOR_CHANNEL, spi_buffer, length);
       theta= (uint8_t)spi_buffer[0];
       cout << "Angle: " <<  unsigned(theta) << endl;
    }
}