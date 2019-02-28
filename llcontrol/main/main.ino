

/* The ESP32 has four SPi buses, however as of right now only two of
 * them are available to use, HSPI and VSPI. Simply using the SPI API 
 * as illustrated in Arduino examples will use VSPI, leaving HSPI unused.
 * 
 * However if we simply intialise two instance of the SPI class for both
 * of these buses both can be used. However when just using these the Arduino
 * way only will actually be outputting at a time.
 * 
 * Logic analyser capture is in the same folder as this example as
 * "multiple_bus_output.png"
 * 
 * created 30/04/2018 by Alistair Symonds
 */
#include "MagAlpha.h"

static const int spiClk = 1000000; // 1 MHz

#define READ_REG_COMMAND    (0b010 << 13)
#define WRITE_REG_COMMAND   (0b100 << 13)

#define VSPI_CS 5
#define HSPI_CS 15

//uninitalised pointers to SPI objects
SPIClass * vspi = NULL;
SPIClass * hspi = NULL;

//MagAlpha
MagAlpha magv;
MagAlpha magh;

void setup() {
  //initialise two instances of the SPIClass attached to VSPI and HSPI respectively
  vspi = new SPIClass(VSPI);
  hspi = new SPIClass(HSPI);
  
  //clock miso mosi ss

  //initialise vspi with default pins
  //SCLK = 18, MISO = 19, MOSI = 23, SS = 5
  //vspi->begin();

  
  // magv.begin(VSPI_CS, vspi);
  // magh.begin(HSPI_CS, hspi);

  

  //alternatively route through GPIO pins of your choice
  //hspi->begin(0, 2, 4, 33); //SCLK, MISO, MOSI, SS
  
  //initialise hspi with default pins
  //SCLK = 14, MISO = 12, MOSI = 13, SS = 15
  hspi->begin(); 
  //alternatively route through GPIO pins
  //hspi->begin(25, 26, 27, 32); //SCLK, MISO, MOSI, SS

  //set up slave select pins as outputs as the Arduino API
  //doesn't handle automatically pulling SS low
  pinMode(VSPI_CS, OUTPUT); //VSPI SS
  pinMode(HSPI_CS, OUTPUT); //HSPI SS

  Serial.begin(115200);
  Serial.println("Hi");

}

// the loop function runs over and over again until power down or reset
void loop() {
  //use the SPI buses
  //double vtheta = vspiRead();
  //double htheta = hspiRead();
  double htheta = hRead();
  Serial.print("v: ");
  //Serial.print(vtheta);
  Serial.print(" | h: ");
  Serial.println(htheta);
  delay(100);
}

double vspiRead() {
  magv.begin(VSPI_CS, vspi);
  return(magv.readAngle());
}

double hspiRead() {
  magh.begin(HSPI_CS, hspi);
  return(magh.readAngle());
}

double hRead(){
    uint16_t angle;
  double angleInDegree;
    digitalWrite(HSPI_CS, LOW);
    angle = hspi->transfer16(0x0000); //Read 16-bit angle
    digitalWrite(HSPI_CS, HIGH);
  angleInDegree = (angle*360.0)/65536.0;
  return angleInDegree;
}
