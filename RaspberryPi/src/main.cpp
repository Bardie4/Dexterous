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
    uint16_t theta1;
    uint16_t theta2;

    uint8_t Theta1;
    uint8_t Theta2;
    uint8_t Theta3;
    uint8_t Theta4;

    double idk;
    //Buffer
    unsigned char spi_buffer[4];
    int length=4;
    while (1){
       spi_buffer[0]=0x00;
       spi_buffer[1]=0x00;
       spi_buffer[2]=0x00;
       spi_buffer[3]=0x00;
       idk=wiringPiSPIDataRW (SENSOR_CHANNEL, spi_buffer, length);
       theta1= (uint16_t) spi_buffer[0] << 8 | spi_buffer [1];
       theta2= (uint16_t) spi_buffer[2] << 8 | spi_buffer [3];
       Theta1= (uint8_t) spi_buffer[0];
       Theta2= (uint8_t) spi_buffer[1];
       Theta3= (uint8_t) spi_buffer[2];
       Theta4= (uint8_t) spi_buffer[3];
       cout << "Angle1: " <<  unsigned(Theta1) << " Angle2: " << unsigned(Theta2) << endl;
    }
}