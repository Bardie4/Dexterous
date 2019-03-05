#include <iostream>
#include <wiringPiSPI.h>
using namespace std;



static const int SENSOR_CHANNEL = 0;
static const int ESP_CHANNEL = 1;
static const int SPI_SCLK_FREQUENCY=10000000;

int main()
{           
    int sensor;
    sensor = wiringPiSPISetup (SENSOR_CHANNEL, SPI_SCLK_FREQUENCY);
    //Angle variable
    double theta;
    //Buffer
    unsigned char spi_buffer[16];
    int length=16;
    while (1){
        spi_buffer[0]=0x0000;
        theta=wiringPiSPIDataRW (SENSOR_CHANNEL, spi_buffer, length):
        cout << "Angle: " << theta << endl;
    }
}