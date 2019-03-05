#include <iostream>
#include <wiringPiSPI.h>
using namespace std;



static const int SENSOR_CHANNEL = 0;
static const int ESP_CHANNEL = 1;
static const int SPI_SCLK_FREQUENCY=10000000;

int main()
{           
    int sensor;
    sensor wiringPiSPISetup (SENSOR_CHANNEL, SPI_SCLK_FREQUENCY);
    //Angle variable
    double theta;
    //Buffer
    uint16_t spi_buffer;
    int length=16;
    while (1){
        spi_buffer=0x0000;
        wiringPiSPIDataRW (channel, spi_buffer, length) ;
        theta=spi_buffer;
        cout << "Angle: " << theta << endl;
    }
}