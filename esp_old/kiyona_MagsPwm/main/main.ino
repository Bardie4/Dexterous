/* asdThe ESP32 has four SPi buses, however as of right now only two of
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
#include <unistd.h>
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

#define TASK_RESET_PERIOD_S     10
#include <PID_v1.h>

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#define H_MOSI  27
#define H_CS    14
#define H_MISO  12
#define H_CLK   13

#define V_MOSI  5
#define V_CS    18
#define V_MISO  19
#define V_CLK   21

#define M1_PWM1 32 //2
#define M1_PWM2 33 //4
#define M1_PWM3 25 //21
#define M1_nFlt 34
#define M1_CH1  0
#define M1_CH2  1
#define M1_CH3  2

#define M2_PWM1 15 //2
#define M2_PWM2 2 //4
#define M2_PWM3 4 //21
#define M2_nFlt 35
#define M2_CH1  3
#define M2_CH2  4
#define M2_CH3  5

#define M_EN    22

#define PWM_FRQ 300000
#define PWM_RES 8

//uninitalised pointers to SPI objects
SPIClass * vspi = NULL;
SPIClass * hspi = NULL;

static const int spiClk = 20*1000*1000; // 20 MHz

// Lookup table
const uint8_t pwmSin[] = {128,131,134,137,140,143,146,149,152,155,158,162,165,167,170,173,176,179,182,185,188,190,193,196,198,201,203,206,208,211,213,215,218,220,222,224,226,228,230,232,234,235,237,238,240,241,243,244,245,246,248,249,250,250,251,252,253,253,254,254,254,255,255,255,255,255,255,255,254,254,254,253,253,252,251,250,250,249,248,246,245,244,243,241,240,238,237,235,234,232,230,228,226,224,222,220,218,215,213,211,208,206,203,201,198,196,193,190,188,185,182,179,176,173,170,167,165,162,158,155,152,149,146,143,140,137,134,131,128,124,121,118,115,112,109,106,103,100,97,93,90,88,85,82,79,76,73,70,67,65,62,59,57,54,52,49,47,44,42,40,37,35,33,31,29,27,25,23,21,20,18,17,15,14,12,11,10,9,7,6,5,5,4,3,2,2,1,1,1,0,0,0,0,0,0,0,1,1,1,2,2,3,4,5,5,6,7,9,10,11,12,14,15,17,18,20,21,23,25,27,29,31,33,35,37,40,42,44,47,49,52,54,57,59,62,65,67,70,73,76,79,82,85,88,90,93,97,100,103,106,109,112,115,118,121,124};

// SPI read -> motor
QueueHandle_t qM1Angle;
QueueHandle_t qM2Angle;

QueueHandle_t qM1Setpoint;
QueueHandle_t qM1Input;
QueueHandle_t qM1Output;


QueueHandle_t qPrintScaling;

// motor write -> Serial write
QueueHandle_t qPrintPWM1A;
QueueHandle_t qPrintPWM1B; 
QueueHandle_t qPrintPWM1C;
QueueHandle_t qPrintPWM2A;
QueueHandle_t qPrintPWM2B; 
QueueHandle_t qPrintPWM2C;
QueueHandle_t qPrintVTheta;  
QueueHandle_t qPrintHTheta;  



void hspiCommand8(void *pvParameters) {
  uint8_t uiAngle, uiAnglePrev;
  int16_t uiAngleDelta;
  double theta, deltaTheta, sumTheta = 0, prevTheta = 0;

  while(1){
    hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
    digitalWrite(H_CS, LOW);
    uiAngle = hspi->transfer(0x00);
    digitalWrite(H_CS, HIGH);
    //theta = (uiAngle*360.0)/65536.0;
    hspi->endTransaction();

    xQueueOverwrite(qM2Angle, &uiAngle);
    xQueueOverwrite(qPrintHTheta, &uiAngle);

    vTaskDelay(10 / portTICK_RATE_MS);
  }
}

void vspiCommand8(void *pvParameters) {
  uint8_t uiAngle, uiAnglePrev;
  int16_t uiAngleDelta;
  double theta; // deltaTheta, sumTheta = 0, prevTheta = 0;
  double Input;

  while (1) {
    vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
    digitalWrite(V_CS, LOW);
    uiAngle = vspi->transfer(0x00);
    digitalWrite(V_CS, HIGH);
    //theta = (uiAngle*360.0)/65536.0;
    vspi->endTransaction();

    uiAngleDelta = uiAngle - uiAnglePrev;
    uiAnglePrev = uiAngle;

    if (uiAngleDelta > 127) uiAngleDelta -= 255;
    else if (uiAngleDelta < -127) uiAngleDelta += 255;
    
    Input = (double)uiAngleDelta;

    xQueueOverwrite(qM1Angle, &uiAngle);
    xQueueOverwrite(qPrintVTheta, &uiAngle);
    xQueueOverwrite(qM1Input, &Input);
 
    TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed = 1;
    TIMERG0.wdt_wprotect = 0;
  }
}

void motor1Control(void *pvParameters) {

  // Angle measurement
  uint8_t theta_raw;
  uint8_t theta_multiplied;
  uint8_t lead_lag;

  // Calculation variables
  uint8_t phaseShift8_120 = 85;
  uint8_t phaseShift8_90 = 64;
  uint8_t phaseShift8_90_minus = 192;
  uint8_t startPos=0;
  double outputScale;
  
  // Initialization values;
  uint8_t theta_0 = 0;
  uint8_t theta_0_mek = 0;

  // PWM
  uint8_t currentStepA = 0;
  uint8_t currentStepB = currentStepA + 85;
  uint8_t currentStepC = currentStepB + 85;
  uint8_t pwmA, pwmB, pwmC;

  // control scaling
  double scaling;

  // Initialize motor driver
  pinMode(M1_PWM1, OUTPUT);
  pinMode(M1_PWM2, OUTPUT);
  pinMode(M1_PWM3, OUTPUT);

  pinMode(M1_nFlt, INPUT);

  // PWM setup
  ledcSetup(M1_CH1, PWM_FRQ, PWM_RES);
  ledcSetup(M1_CH2, PWM_FRQ, PWM_RES);
  ledcSetup(M1_CH3, PWM_FRQ, PWM_RES);

  ledcAttachPin(M1_PWM1, M1_CH1);
  ledcAttachPin(M1_PWM2, M1_CH2);
  ledcAttachPin(M1_PWM3, M1_CH3);

  // Initial PWM value
  pwmA = (pwmSin[currentStepA]);
  pwmB = (pwmSin[currentStepB]);
  pwmC = (pwmSin[currentStepC]);

  ledcWrite(M1_CH1, 0);
  ledcWrite(M1_CH2, 85);
  ledcWrite(M1_CH3, 170);

  vTaskDelay(2000 / portTICK_RATE_MS);

  Serial.println("start M1");
  xQueuePeek(qM1Angle, &theta_0_mek, 0);

  while(1){    
    // Read angle
    xQueuePeek(qM1Angle, &theta_raw, 0);
    xQueuePeek(qM1Output, &scaling, 0);

    //Leading or lagging electrical field
    if (scaling <= 0) lead_lag = phaseShift8_90;
    else lead_lag = phaseShift8_90_minus;

    scaling/=100.0;

    // Find optimal electrical angle
    theta_raw += (255 - theta_0_mek);
    theta_multiplied = theta_raw << 2;        //Multiplied by 4 for electrical angle.
    theta_multiplied += lead_lag;             //Adding lead/lag angle based on direction
    currentStepA = theta_multiplied;          //Masked by conversion back to 16 bit to cycle around 360 deg 4 times in one mechanical rotation. (Modulus 360 degrees)
    currentStepB = currentStepA + phaseShift8_120;
    currentStepC = currentStepB + phaseShift8_120;

    //Output
    pwmA = (int)(pwmSin[currentStepA] * scaling);
    pwmB = (int)(pwmSin[currentStepB] * scaling);
    pwmC = (int)(pwmSin[currentStepC] * scaling);

    ledcWrite(M1_CH1, pwmA);
    ledcWrite(M1_CH2, pwmB);
    ledcWrite(M1_CH3, pwmC);
    
    xQueueOverwrite(qPrintPWM1A, &pwmA);
    xQueueOverwrite(qPrintPWM1B, &pwmB);
    xQueueOverwrite(qPrintPWM1C, &pwmC);

    xQueueOverwrite(qPrintScaling, &scaling);
    
    TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed = 1;
    TIMERG0.wdt_wprotect = 0;
  }
}

void motor2Control(void *pvParameters) {
  // Angle measurement
  uint8_t theta_raw;
  uint8_t theta_multiplied;
  uint8_t lead_lag;

  // Calculation variables
  uint8_t phaseShift8_120 = 85;
  uint8_t phaseShift8_90 = 64;
  uint8_t phaseShift8_90_minus = 192;
  uint8_t startPos=0;
  double outputScale;
  
  // Initialization values;
  uint8_t theta_0 = 0;
  uint8_t theta_0_mek = 0;

  // PWM
  uint8_t currentStepA = 0;
  uint8_t currentStepB = currentStepA + 85;
  uint8_t currentStepC = currentStepB + 85;
  uint8_t pwmA, pwmB, pwmC;

  // control scaling
  double scaling;
  double uScale = 0;

  // Initialize motor driver
  pinMode(M2_PWM1, OUTPUT);
  pinMode(M2_PWM2, OUTPUT);
  pinMode(M2_PWM3, OUTPUT);

  pinMode(M2_nFlt, INPUT);

  // PWM setup
  ledcSetup(M2_CH1, PWM_FRQ, PWM_RES);
  ledcSetup(M2_CH2, PWM_FRQ, PWM_RES);
  ledcSetup(M2_CH3, PWM_FRQ, PWM_RES);

  ledcAttachPin(M2_PWM1, M2_CH1);
  ledcAttachPin(M2_PWM2, M2_CH2);
  ledcAttachPin(M2_PWM3, M2_CH3);

  // Initial PWM value
  pwmA = (pwmSin[currentStepA]);
  pwmB = (pwmSin[currentStepB]);
  pwmC = (pwmSin[currentStepC]);

  ledcWrite(M2_CH1, 0);
  ledcWrite(M2_CH2, 85);
  ledcWrite(M2_CH3, 170);

  vTaskDelay(2000 / portTICK_RATE_MS);
  
  Serial.println("start M1");
  xQueuePeek(qM2Angle, &theta_0_mek, 0);

  while(1){
    //Leading or lagging electrical field
    // if (u <= 0) lead_lag = phaseShift8_90;
    // else if (u > 0) lead_lag = phaseShift8_90_minus;
    
    // Read angle
    xQueuePeek(qM2Angle, &theta_raw, 0);

    // Find optimal electrical angle
    theta_raw += (255 - theta_0_mek);
    theta_multiplied = theta_raw << 2;        //Multiplied by 4 for electrical angle.
    theta_multiplied += lead_lag;             //Adding lead/lag angle based on direction
    currentStepA = theta_multiplied;          //Masked by conversion back to 16 bit to cycle around 360 deg 4 times in one mechanical rotation. (Modulus 360 degrees)
    currentStepB = currentStepA + phaseShift8_120;
    currentStepC = currentStepB + phaseShift8_120;

    //Output

    pwmA = (int)(pwmSin[currentStepA]);
    pwmB = (int)(pwmSin[currentStepB]);
    pwmC = (int)(pwmSin[currentStepC]);

    ledcWrite(M2_CH1, pwmA);
    ledcWrite(M2_CH2, pwmB);
    ledcWrite(M2_CH3, pwmC);
    
    xQueueOverwrite(qPrintPWM2A, &pwmA);
    xQueueOverwrite(qPrintPWM2B, &pwmB);
    xQueueOverwrite(qPrintPWM2C, &pwmC);
      
    TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed = 1;
    TIMERG0.wdt_wprotect = 0;
  }
}


void printer(void *pvParameters) {
  uint8_t hTheta, vTheta;
  uint8_t pwm1A, pwm1B, pwm1C, pwm2A, pwm2B, pwm2C; 
  double scaling;

  while(1){
    xQueuePeek( qPrintVTheta, &vTheta, 0 );
    xQueuePeek( qPrintPWM1A, &pwm1A, 0 );
    xQueuePeek( qPrintPWM1B, &pwm1B, 0 );
    xQueuePeek( qPrintPWM1C, &pwm1C, 0 );

    xQueuePeek( qPrintScaling, &scaling, 0);

    xQueuePeek( qPrintHTheta, &hTheta, 0 );
    xQueuePeek( qPrintPWM2A, &pwm2A, 0 );
    xQueuePeek( qPrintPWM2B, &pwm2B, 0 );
    xQueuePeek( qPrintPWM2C, &pwm2C, 0 );

    Serial.print(" | vTheta: ");
    Serial.print(vTheta);
    Serial.print(" | PWM1 : ");
    Serial.print(pwm1A);
    Serial.print(" | ");
    Serial.print(pwm1B);
    Serial.print(" | ");
    Serial.print(pwm1C);
    Serial.print(" | scale: ");
    Serial.print(pwm1C);    

    Serial.print(" | hTheta: ");
    Serial.print(hTheta);
    Serial.print(" | PWM2 : ");
    Serial.print(pwm2A);
    Serial.print(" | ");
    Serial.print(pwm2B);
    Serial.print(" | ");
    Serial.println(pwm2C);

    vTaskDelay(100 / portTICK_RATE_MS);
  }
  
}


void motor1PID(void *pvParameters)  {
  double received[3];
  //Specify the initial tuning parameters
  double Input, Output, Setpoint;
  double Kp = 1.00, Ki = 0.00, Kd = 0.00;
  PID pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
  pid.SetOutputLimits(-100.0, 100.0);

  while(1){
    xQueuePeek( qM1Angle, &Input, 0);
    xQueuePeek( qM1Setpoint, &Setpoint, 0);

    if( Serial.available() > 0) {
      Setpoint = Serial.parseFloat();
      Kp = Serial.parseFloat();

      Serial.print("St: ");
      Serial.print(Setpoint);
      Serial.print(" Kp: ");
      Serial.println(Kp);
    }

    pid.Compute();
    xQueueOverwrite(qM1Output, &Output);
    
    vTaskDelay(10 / portTICK_RATE_MS);
  }
  
}

void motor1MasterCommand(void *pvParameters){
  while(1){
    double Setpoint;

    // Read SPI
    // hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
    // digitalWrite(H_CS, LOW);
    // uiAngle = hspi->transfer(0x00);
    // digitalWrite(H_CS, HIGH);
    // theta = (uiAngle*360.0)/65536.0;
    // hspi->endTransaction();

    // xQueueOverwrite( qM1Setpoint, &Setpoint)

    vTaskDelay(100 / portTICK_RATE_MS);
  
  }  
}

void motor1Scroll(void *pvParameters) {
  uint8_t phaseShift8_120 = 256/3;
  uint8_t currentStepA = 0;
  uint8_t currentStepB = currentStepA + 85;
  uint8_t currentStepC = currentStepB + 85;
  //pwm
  uint8_t pwmA, pwmB, pwmC;

  pinMode(M1_PWM1, OUTPUT);
  pinMode(M1_PWM2, OUTPUT);
  pinMode(M1_PWM3, OUTPUT);

    // PWM
  ledcSetup(M1_CH1, PWM_FRQ, PWM_RES);
  ledcSetup(M1_CH2, PWM_FRQ, PWM_RES);
  ledcSetup(M1_CH3, PWM_FRQ, PWM_RES);

  ledcAttachPin(M1_PWM1, M1_CH1);
  ledcAttachPin(M1_PWM2, M1_CH2);
  ledcAttachPin(M1_PWM3, M1_CH3);

  while(1){
    // Scroll through field
    for(uint8_t i = 0; i < 255; i++)
    {
      currentStepA = i;          //Masked by conversion back to 16 bit to cycle around 360 deg 4 times in one mechanical rotation. (Modulus 360 degrees)
      currentStepB = currentStepA + phaseShift8_120;
      currentStepC = currentStepB + phaseShift8_120;
      pwmA = (int)(pwmSin[currentStepA]);
      pwmB = (int)(pwmSin[currentStepB]);
      pwmC = (int)(pwmSin[currentStepC]);
      ledcWrite(M1_CH1, pwmA);
      ledcWrite(M1_CH2, pwmB);
      ledcWrite(M1_CH3, pwmC);
  
      // xQueueOverwrite(qPrintPWM2A, &pwmA);
      // xQueueOverwrite(qPrintPWM2B, &pwmB);
      // xQueueOverwrite(qPrintPWM2C, &pwmC);
      usleep(100);
    }
  }
}

void motor2Scroll(void *pvParameters) {
  uint8_t phaseShift8_120 = 256/3;
  uint8_t currentStepA = 0;
  uint8_t currentStepB = currentStepA + 85;
  uint8_t currentStepC = currentStepB + 85;
  //pwm
  uint8_t pwmA, pwmB, pwmC;

  pinMode(M2_PWM1, OUTPUT);
  pinMode(M2_PWM2, OUTPUT);
  pinMode(M2_PWM3, OUTPUT);

  //PWM
  ledcSetup(M2_CH1, PWM_FRQ, PWM_RES);
  ledcSetup(M2_CH2, PWM_FRQ, PWM_RES);
  ledcSetup(M2_CH3, PWM_FRQ, PWM_RES);

  ledcAttachPin(M2_PWM1, M2_CH1);
  ledcAttachPin(M2_PWM2, M2_CH2);
  ledcAttachPin(M2_PWM3, M2_CH3);


  while(1){
    // Scroll throug field
    for(uint8_t i = 0; i < 255; i++)
    {
      currentStepA = i;          //Masked by conversion back to 16 bit to cycle around 360 deg 4 times in one mechanical rotation. (Modulus 360 degrees)
      currentStepB = currentStepA + phaseShift8_120;
      currentStepC = currentStepB + phaseShift8_120;
      pwmA = (int)(pwmSin[currentStepA]);
      pwmB = (int)(pwmSin[currentStepB]);
      pwmC = (int)(pwmSin[currentStepC]);
      ledcWrite(M2_CH1, pwmA);
      ledcWrite(M2_CH2, pwmB);
      ledcWrite(M2_CH3, pwmC);

      usleep(100);
    }
  }
}

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

  pinMode(M_EN, OUTPUT);
  digitalWrite(M_EN, HIGH);

  // Angle read -> motor write
  qM1Angle = xQueueCreate( 1, sizeof( uint8_t ) );
  qM2Angle = xQueueCreate( 1, sizeof( uint8_t ) );

  qM1Setpoint = xQueueCreate( 1, sizeof( double ) );
  qM1Input = xQueueCreate( 1, sizeof ( double ) );
  qM1Output = xQueueCreate( 1, sizeof( double ) );

  qPrintScaling = xQueueCreate( 1, sizeof( double ) );

  // motor write -> Serial write
  qPrintPWM1A = xQueueCreate( 1, sizeof( uint8_t ) );
  qPrintPWM1B = xQueueCreate( 1, sizeof( uint8_t ) );
  qPrintPWM1C = xQueueCreate( 1, sizeof( uint8_t ) );
  qPrintPWM2A = xQueueCreate( 1, sizeof( uint8_t ) );
  qPrintPWM2B = xQueueCreate( 1, sizeof( uint8_t ) );
  qPrintPWM2C = xQueueCreate( 1, sizeof( uint8_t ) );
  qPrintVTheta = xQueueCreate( 1, sizeof( uint8_t ) );
  qPrintHTheta = xQueueCreate( 1, sizeof( uint8_t ) );


  xTaskCreatePinnedToCore(vspiCommand8, "vspi", 4096, (void *)1, 1, NULL, 1);
  //xTaskCreatePinnedToCore(motor1Control, "M1Ctrl", 4096, (void *)1, 1, NULL, 0);
  xTaskCreatePinnedToCore(hspiCommand8, "hspi", 4096, (void *)2, 1, NULL, 1);
  //xTaskCreatePinnedToCore(motor2Control, "M2Ctrl", 4096, (void *)1, 1, NULL, 1);
  //xTaskCreatePinnedToCore(motor1PID, "M1PID", 4096, (void *)1, 1, NULL, 1);
  //xTaskCreatePinnedToCore(motor1MasterCommand, "passMaster", 4096, (void *)2, 1, NULL, 1);
  xTaskCreatePinnedToCore(motor1Scroll, "M1_scroll", 4096, (void *)1, 1, NULL, 1);
  xTaskCreatePinnedToCore(motor2Scroll, "M2_scroll", 4096, (void *)1, 1, NULL, 1);
  xTaskCreatePinnedToCore(printer, "printer", 4096, (void *)1, 1, NULL, 0);

  Serial.begin(115200);
}

// the loop function runs over and over again until power down or reset
void loop() {
  vTaskDelay(portMAX_DELAY);
}



// void vspiCommand16(void *pvParameters) {
//   uint16_t uiAngle;
//   double theta, deltaTheta, sumTheta = 0, prevTheta = 0;

//   while (1) {

//     vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
//     digitalWrite(5, LOW);
//     uiAngle = vspi->transfer16(0x0000);
//     digitalWrite(5, HIGH);
//     theta = (uiAngle*360.0)/65536.0;
//     vspi->endTransaction();

//     // Angle change
//     deltaTheta = theta - prevTheta;

//     // Integrate (position)
//     if (deltaTheta > 180)
//     {
//       sumTheta += (deltaTheta - 360);
//     }
//     else if (deltaTheta < -180){
//       sumTheta += (360 + deltaTheta);
//     } 
//     else{
//       sumTheta += deltaTheta;
//     }
    
//     // Previous angle
//     prevTheta = theta;

//     xQueueOverwrite( qMotorAngle, &theta);
//     xQueueOverwrite( qMotorSumAngle, &sumTheta );

//     xQueueOverwrite( qPrintVTheta, &theta);
//     xQueueOverwrite( qPrintVSumTheta, &sumTheta );
    
//     vTaskDelay(1 / portTICK_RATE_MS);
//   }
// }

// void hspiCommand16(void *pvParameters) {
//   uint16_t uiAngle;
//   double theta, deltaTheta, sumTheta = 0, prevTheta = 0;
//   while(1){
//     hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
//     digitalWrite(15, LOW);
//     uiAngle = hspi->transfer16(0x0000);
//     digitalWrite(15, HIGH);
//     theta = (uiAngle*360.0)/65536.0;
//     hspi->endTransaction();

//     // Angle change
//     deltaTheta = theta - prevTheta;

//     // Integrate (position)
//     if (deltaTheta > 180)
//     {
//       sumTheta += (deltaTheta - 360);
//     }
//     else if (deltaTheta < -180){
//       sumTheta += (360 + deltaTheta);
//     } 
//     else{
//       sumTheta += deltaTheta;
//     }
    
//     // Previous angle
//     prevTheta = theta;

//     xQueueOverwrite( qPrintHTheta, &theta);
//     xQueueOverwrite( qPrintHSumTheta, &sumTheta );

//     vTaskDelay(1 / portTICK_RATE_MS);
//   }
// }
