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
#include <PID_v1.h>

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

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
double probe;
double time_loop;

//Specify the links and initial tuning parameters
double Kp = 0.8, Ki = 0.2, Kd = 0.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);

int start;
int phaseShift;

const int pwmSin[] = {128, 132, 136, 140, 143, 147, 151, 155, 159, 162, 166, 170, 174, 178, 181, 185, 189, 192, 196, 200, 203, 207, 211, 214, 218, 221, 225, 228, 232, 235, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 235, 232, 228, 225, 221, 218, 214, 211, 207, 203, 200, 196, 192, 189, 185, 181, 178, 174, 170, 166, 162, 159, 155, 151, 147, 143, 140, 136, 132, 128, 124, 120, 116, 113, 109, 105, 101, 97, 94, 90, 86, 82, 78, 75, 71, 67, 64, 60, 56, 53, 49, 45, 42, 38, 35, 31, 28, 24, 21, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21, 24, 28, 31, 35, 38, 42, 45, 49, 53, 56, 60, 64, 67, 71, 75, 78, 82, 86, 90, 94, 97, 101, 105, 109, 113, 116, 120, 124};

// SPI read -> motor
QueueHandle_t qMotorAngle;
QueueHandle_t qMotorSumAngle;

// motor write -> Serial write
QueueHandle_t qPrintOutput ;  
QueueHandle_t qPrintSetpoint ;  
QueueHandle_t qPrintPWMA ;
QueueHandle_t qPrintPWMB ; 
QueueHandle_t qPrintPWMC ;
QueueHandle_t qPrintMTheta ;  
QueueHandle_t qPrintMSumTheta ;  
QueueHandle_t qPrintVTheta ;  
QueueHandle_t qPrintVSumTheta ;  
QueueHandle_t qPrintHTheta ;  
QueueHandle_t qPrintHSumTheta ;  


void setup() {
  //initialise two instances of the SPIClass attached to VSPI and HSPI respectively
  vspi = new SPIClass(VSPI);
  hspi = new SPIClass(HSPI);

  //initialise vspi with default pins
  //SCLK = 18, MISO = 19, MOSI = 23, SS = 5
  vspi->begin(V_CLK, V_MISO, V_MOSI, V_CS);
  //SCLK = 14, MISO = 12, MOSI = 13, SS = 15
  hspi->begin(H_CLK, H_MISO, H_MOSI, H_CS); 

  pinMode(V_CS, OUTPUT); //VSPI SS
  pinMode(H_CS, OUTPUT); //HSPI SS
  
  mot_eventgroup = xEventGroupCreate();

  // Angle read -> motor write
  qMotorAngle = xQueueCreate( 1, sizeof( double ) );
  qMotorSumAngle = xQueueCreate( 1, sizeof( double ) );

  // motor write -> Serial write
  qPrintOutput = xQueueCreate( 1, sizeof( double ) );
  qPrintSetpoint = xQueueCreate( 1, sizeof( double ) );
  qPrintPWMA = xQueueCreate( 1, sizeof( int ) );
  qPrintPWMB = xQueueCreate( 1, sizeof( int ) );
  qPrintPWMC = xQueueCreate( 1, sizeof( int ) );
  qPrintMTheta = xQueueCreate( 1, sizeof( double ) );
  qPrintMSumTheta = xQueueCreate( 1, sizeof( double ) );
  qPrintVTheta = xQueueCreate( 1, sizeof( uint8_t ) );
  qPrintVSumTheta = xQueueCreate( 1, sizeof( double ) );
  qPrintHTheta = xQueueCreate( 1, sizeof( uint8_t ) );
  qPrintHSumTheta = xQueueCreate( 1, sizeof( double ) );

  // xTaskCreatePinnedToCore(vspiCommand16, "vspi", 4096, (void *)1, 1, NULL, 0);
  // xTaskCreatePinnedToCore(hspiCommand16, "hspi", 4096, (void *)2, 1, NULL, 1);
  xTaskCreatePinnedToCore(vspiCommand8, "vspi", 4096, (void *)1, 1, NULL, 0);
  xTaskCreatePinnedToCore(hspiCommand8, "hspi", 4096, (void *)2, 1, NULL, 1);
  xTaskCreatePinnedToCore(M1_ctrl, "M1_ctrl", 4096, (void *)1, 1, NULL, 0);
  xTaskCreatePinnedToCore(printer, "printer", 4096, (void *)1, 1, NULL, 1);

  Serial.begin(115200);
}

// the loop function runs over and over again until power down or reset
void loop() {

}

void vspiCommand16(void *pvParameters) {
  uint16_t uiAngle;
  double theta, deltaTheta, sumTheta = 0, prevTheta = 0;

  while (1) {

    vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
    digitalWrite(5, LOW);
    uiAngle = vspi->transfer16(0x0000);
    digitalWrite(5, HIGH);
    theta = (uiAngle*360.0)/65536.0;
    vspi->endTransaction();

    // Angle change
    deltaTheta = theta - prevTheta;

    // Integrate (position)
    if (deltaTheta > 180)
    {
      sumTheta += (deltaTheta - 360);
    }
    else if (deltaTheta < -180){
      sumTheta += (360 + deltaTheta);
    } 
    else{
      sumTheta += deltaTheta;
    }
    
    // Previous angle
    prevTheta = theta;

    xQueueOverwrite( qMotorAngle, &theta);
    xQueueOverwrite( qMotorSumAngle, &sumTheta );

    xQueueOverwrite( qPrintVTheta, &theta);
    xQueueOverwrite( qPrintVSumTheta, &sumTheta );
    
    vTaskDelay(1 / portTICK_RATE_MS);
  }
}

void hspiCommand16(void *pvParameters) {
  uint16_t uiAngle;
  double theta, deltaTheta, sumTheta = 0, prevTheta = 0;
  while(1){
    hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
    digitalWrite(15, LOW);
    uiAngle = hspi->transfer16(0x0000);
    digitalWrite(15, HIGH);
    theta = (uiAngle*360.0)/65536.0;
    hspi->endTransaction();

    // Angle change
    deltaTheta = theta - prevTheta;

    // Integrate (position)
    if (deltaTheta > 180)
    {
      sumTheta += (deltaTheta - 360);
    }
    else if (deltaTheta < -180){
      sumTheta += (360 + deltaTheta);
    } 
    else{
      sumTheta += deltaTheta;
    }
    
    // Previous angle
    prevTheta = theta;

    xQueueOverwrite( qPrintHTheta, &theta);
    xQueueOverwrite( qPrintHSumTheta, &sumTheta );

    vTaskDelay(1 / portTICK_RATE_MS);
  }
}

void vspiCommand8(void *pvParameters) {
  uint8_t uiAngle;
  double theta, deltaTheta, sumTheta = 0, prevTheta = 0;

  while (1) {
    vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
    digitalWrite(5, LOW);
    uiAngle = vspi->transfer(0b00000000);
    digitalWrite(5, HIGH);
    theta = (uiAngle*360.0)/65536.0;
    vspi->endTransaction();

    xQueueOverwrite( qPrintVTheta, &uiAngle);

    vTaskDelay(1 / portTICK_RATE_MS);
  }
}

void hspiCommand8(void *pvParameters) {
  uint8_t uiAngle;
  double theta, deltaTheta, sumTheta = 0, prevTheta = 0;

  while(1){
    hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
    digitalWrite(15, LOW);
    uiAngle = hspi->transfer(0b00000000);
    digitalWrite(15, HIGH);
    theta = (uiAngle*360.0)/65536.0;
    hspi->endTransaction();

    xQueueOverwrite( qPrintHTheta, &uiAngle);

    vTaskDelay(1 / portTICK_RATE_MS);
  }
}

void M1_ctrl(void *pvParameters) {
  int lead_lag;
  double e_theta_0;
  int currentStepA;
  int currentStepB;
  int currentStepC;
  int pwmA, pwmB, pwmC;
  int sineArraySize;
  int increment = 0;
  double e_theta; //electrical angle
  double a_theta; //array angle
  boolean direct = 1; // direction true=forward, false=backward
  int read_pot;
  //90 degrees angle;
  int ea_nintey = sineArraySize/4;

  const int freq = 250000;
  const int resolution = 8;

  // Read angle
  double sumTheta, theta;

  // PV
  double output;
 
  // configure LED PWM functionalitites
  sigmaDeltaSetup(0, freq);
  sigmaDeltaSetup(1, freq);
  sigmaDeltaSetup(2, freq);

  // attach the channel to the GPIO to be controlled
  sigmaDeltaAttachPin(M1_PWM1, 0);
  sigmaDeltaAttachPin(M1_PWM2, 1);
  sigmaDeltaAttachPin(M1_PWM3, 2);

  // PID
  Setpoint = 80;
  myPID.SetOutputLimits(-100,100);
  myPID.SetMode(AUTOMATIC);
  
  pinMode(M1_PWM1, OUTPUT); 
  pinMode(M1_PWM2, OUTPUT); 
  pinMode(M1_PWM3, OUTPUT); 
  
  pinMode(M1_EN, OUTPUT); 

  digitalWrite(M1_EN, HIGH);

  sineArraySize = sizeof(pwmSin) / sizeof(int); // Find lookup table size 
  int phaseShift = sineArraySize / 3.0; 
  currentStepA = 0;
  currentStepB = currentStepA + phaseShift;
  currentStepC = currentStepB + phaseShift;
 
  sineArraySize--; // Convert from array Size to last PWM array number

  while(1){
    xQueuePeek( qMotorSumAngle, &sumTheta, 0 );
    xQueuePeek( qMotorAngle, &theta, 0 );

    // PID
    Input = sumTheta;

    e_theta = 4.0 * theta - e_theta_0;
    if (e_theta < 0.0) e_theta = 360.0 + e_theta;
    else if (e_theta > 360.0 && e_theta <= 720.0) e_theta = e_theta - 360.0;
    else if (e_theta > 720.0 && e_theta <= 1080.0) e_theta = e_theta - 720.0;
    else if (e_theta > 1080.0) e_theta = e_theta - 1080.0;

    if (start == 1){
      e_theta_0 = e_theta;
      start = 0;
      e_theta = 0.0;
    }

    a_theta = floor((sineArraySize/360.0) * e_theta);
    
    myPID.Compute(); // Update Output
    
    ea_nintey = sineArraySize/4.0;
    if (Output < 0)
      lead_lag = -ea_nintey;
    else if (Output >= 0)
      lead_lag = ea_nintey;
    
    // PWM
    output = abs(Output) / 100.0; // 10 - (10+x)%

    currentStepA = a_theta + lead_lag;
    currentStepB = currentStepA + phaseShift;
    currentStepC = currentStepB + phaseShift;

    if(currentStepA > sineArraySize)  currentStepA -= sineArraySize;
    else if(currentStepA < 0)  currentStepA += sineArraySize;
 
    if(currentStepB > sineArraySize)  currentStepB -= sineArraySize;
    else if(currentStepB < 0)  currentStepB += sineArraySize;
 
    if(currentStepC > sineArraySize)  currentStepC -= sineArraySize;
    else if(currentStepC < 0) currentStepC += sineArraySize;


    // WRITE PWM
    // analogWrite(M1_PWM1, pwmSin[currentStepA]*u);
    // analogWrite(M1_PWM2, pwmSin[currentStepB]*u);
    // analogWrite(M1_PWM3, pwmSin[currentStepC]*u);  
    pwmA = ((int)(pwmSin[currentStepA])*output);
    pwmB = ((int)(pwmSin[currentStepB])*output);
    pwmC = ((int)(pwmSin[currentStepC])*output);
    sigmaDeltaWrite(0, pwmA);
    sigmaDeltaWrite(1, pwmB);
    sigmaDeltaWrite(2, pwmC);

    xQueueOverwrite( qPrintPWMA, &pwmA);
    xQueueOverwrite( qPrintPWMB, &pwmB );
    xQueueOverwrite( qPrintPWMC, &pwmC );
    xQueueOverwrite( qPrintOutput, &output);
    xQueueOverwrite( qPrintSetpoint, &Setpoint );
    xQueueOverwrite( qPrintMTheta, &theta );
    xQueueOverwrite( qPrintMSumTheta, &sumTheta);

    vTaskDelay(1 / portTICK_RATE_MS);
    }
}

void printer(void *pvParameters) {
  double output, Setpoint, vSumTheta, hSumTheta, mTheta, mSumTheta;
  uint8_t hTheta, vTheta;
  int pwmA, pwmB, pwmC; 

  while(1){
    xQueuePeek( qPrintOutput, &output, 0 );
    xQueuePeek( qPrintSetpoint, &Setpoint, 0 );
    xQueuePeek( qPrintMTheta, &mTheta, 0 );
    xQueuePeek( qPrintMSumTheta, &mSumTheta, 0 );
    xQueuePeek( qPrintVTheta, &vTheta, 0 );
    xQueuePeek( qPrintVSumTheta, &vSumTheta, 0 );
    xQueuePeek( qPrintHTheta, &hTheta, 0 );
    xQueuePeek( qPrintHSumTheta, &hSumTheta, 0 );
    xQueuePeek( qPrintPWMA, &pwmA, 0 );
    xQueuePeek( qPrintPWMB, &pwmB, 0 );
    xQueuePeek( qPrintPWMC, &pwmC, 0 );

    // Serial.print("Output : ");
    // Serial.print(output);
    // Serial.print(" | Set: ");
    // Serial.print(Setpoint);
    Serial.print(" | vTheta: ");
    Serial.print(vTheta);
    // Serial.print(" | vSumTheta: ");
    // Serial.print(vSumTheta);
    Serial.print(" | hTheta: ");
    Serial.print(hTheta);
    // Serial.println(" | hSumTheta: ");
    // Serial.print(hSumTheta);
    Serial.print(" | mTheta: ");
    Serial.println(mTheta);
    // Serial.print(" | mSumTheta: ");
    // Serial.print(mSumTheta);

    // Serial.print(" | PWM :");
    // Serial.print(pwmA);
    // Serial.print(" | ");
    // Serial.print(pwmB);
    // Serial.print(" | ");
    // Serial.println(pwmC);

    vTaskDelay(100 / portTICK_RATE_MS);
  }
  
}