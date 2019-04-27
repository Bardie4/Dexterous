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

#include <SPI.h>
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

xQueueHandle mot1_queue;
xQueueHandle mot2_queue;
EventGroupHandle_t mot_eventgroup;

static const int spiClk = 10*1000*1000; // 10 MHz

//uninitalised pointers to SPI objects
SPIClass * vspi = NULL;
SPIClass * hspi = NULL;

#define H_MOSI  13
#define H_MISO  12
#define H_CLK   14
#define H_CS    15

#define V_MOSI  23
#define V_MISO  19
#define V_CLK   18
#define V_CS    5

#define M1_PWM1 0
#define M1_PWM2 2
#define M1_PWM3 4
#define M1_EN   16

void setup() {
  //initialise two instances of the SPIClass attached to VSPI and HSPI respectively
  vspi = new SPIClass(VSPI);
  hspi = new SPIClass(HSPI);
  
  //clock miso mosi ss

  //initialise vspi with default pins
  //SCLK = 18, MISO = 19, MOSI = 23, SS = 5
  vspi->begin(V_CLK, V_MISO, V_MOSI, V_CS);
  //alternatively route through GPIO pins of your choice
  //hspi->begin(0, 2, 4, 33); //SCLK, MISO, MOSI, SS
  
  //initialise hspi with default pins
  //SCLK = 14, MISO = 12, MOSI = 13, SS = 15
  hspi->begin(H_CLK, H_MISO, H_MOSI, H_CS); 
  //alternatively route through GPIO pins
  //hspi->begin(25, 26, 27, 32); //SCLK, MISO, MOSI, SS

  //set up slave select pins as outputs as the Arduino API
  //doesn't handle automatically pulling SS low
  pinMode(V_CS, OUTPUT); //VSPI SS
  pinMode(H_CS, OUTPUT); //HSPI SS
  
  Serial.begin(115200);

  mot_eventgroup = xEventGroupCreate();

  xTaskCreatePinnedToCore(vspiCommand, "vspi", 4096, (void *)1, 1, NULL, 0);
  xTaskCreatePinnedToCore(hspiCommand, "hspi", 4096, (void *)2, 1, NULL, 1);

}

// the loop function runs over and over again until power down or reset
void loop() {

}

void vspiCommand(void *pvParameters) {
  uint16_t angle;
  double angleInDegree;
  while (1) {
    unsigned long start_time = micros();
    vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
    digitalWrite(5, LOW);
    angle = vspi->transfer16(0x0000);
    digitalWrite(5, HIGH);
    angleInDegree = (angle*360.0)/65536.0;
    vspi->endTransaction();
    // Serial.print("            vT: ");
    // Serial.print(angleInDegree);
    // Serial.print(" core: ");
    // Serial.print(xPortGetCoreID());
    // Serial.print(" Time: ");
    Serial.print(micros() - start_time);
    Serial.println(" us");
    vTaskDelay(100 / portTICK_RATE_MS);
  }
}

void hspiCommand(void *pvParameters) {
  uint16_t angle;
  double angleInDegree;
  while(1){
    unsigned long start_time = micros();
    hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
    digitalWrite(15, LOW);
    angle = hspi->transfer16(0x0000);
    digitalWrite(15, HIGH);
    angleInDegree = (angle*360.0)/65536.0;
    hspi->endTransaction();
    // Serial.print("hT: ");
    // Serial.print(angleInDegree);
    // Serial.print(" core: ");
    // Serial.print(xPortGetCoreID());
    // Serial.print(" Time: ");
    Serial.print(micros() - start_time);
    Serial.println(" us");
    vTaskDelay(100 / portTICK_RATE_MS);
  }
}
