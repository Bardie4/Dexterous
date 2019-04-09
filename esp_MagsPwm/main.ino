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
#include "esp_task_wdt.h"

#define TASK_RESET_PERIOD_S     10
//#include <PID_v1.h>

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#define H_MOSI  13
#define H_MISO  12
#define H_CLK   14
#define H_CS    15

#define V_MOSI  23
#define V_MISO  19
#define V_CLK   18
#define V_CS    5

#define M1_PWM1 25 //2
#define M1_PWM2 33 //4
#define M1_PWM3 32 //21
#define M1_nRst 26
#define M1_nSlp 27
#define M1_EN   16
#define M1_CH1  0
#define M1_CH2  1
#define M1_CH3  2

#define M2_PWM1 2 //2
#define M2_PWM2 4 //4
#define M2_PWM3 21 //21
#define M2_nRst 26
#define M2_nSlp 27
#define M2_EN   16
#define M2_CH1  3
#define M2_CH2  4
#define M2_CH3  5

#define PWM_FRQ 30000
#define PWM_RES 8

xQueueHandle mot1_queue;
xQueueHandle mot2_queue;
EventGroupHandle_t mot_eventgroup;

//uninitalised pointers to SPI objects
SPIClass * vspi = NULL;
SPIClass * hspi = NULL;

static const int spiClk = 20*1000*1000; // 20 MHz

// //Define Variables we'll be connecting to
// double Setpoint, Input, Output;
// double probe;
// double time_loop;

// //Specify the links and initial tuning parameters
// double Kp = 0.8, Ki = 0.2, Kd = 0.0;
// PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);

int start;
int phaseShift;

const uint8_t pwmSin[] = {128,131,134,137,140,143,146,149,152,155,158,162,165,167,170,173,176,179,182,185,188,190,193,196,198,201,203,206,208,211,213,215,218,220,222,224,226,228,230,232,234,235,237,238,240,241,243,244,245,246,248,249,250,250,251,252,253,253,254,254,254,255,255,255,255,255,255,255,254,254,254,253,253,252,251,250,250,249,248,246,245,244,243,241,240,238,237,235,234,232,230,228,226,224,222,220,218,215,213,211,208,206,203,201,198,196,193,190,188,185,182,179,176,173,170,167,165,162,158,155,152,149,146,143,140,137,134,131,128,124,121,118,115,112,109,106,103,100,97,93,90,88,85,82,79,76,73,70,67,65,62,59,57,54,52,49,47,44,42,40,37,35,33,31,29,27,25,23,21,20,18,17,15,14,12,11,10,9,7,6,5,5,4,3,2,2,1,1,1,0,0,0,0,0,0,0,1,1,1,2,2,3,4,5,5,6,7,9,10,11,12,14,15,17,18,20,21,23,25,27,29,31,33,35,37,40,42,44,47,49,52,54,57,59,62,65,67,70,73,76,79,82,85,88,90,93,97,100,103,106,109,112,115,118,121,124};

// SPI read -> motor
QueueHandle_t qMotorAngle;
QueueHandle_t qMotorSumAngle;

// motor write -> Serial write
QueueHandle_t qPrintOutput;  
QueueHandle_t qPrintSetpoint;  
QueueHandle_t qPrintPWMA;
QueueHandle_t qPrintPWMB; 
QueueHandle_t qPrintPWMC;
QueueHandle_t qPrintMTheta;  
QueueHandle_t qPrintMSumTheta;  
QueueHandle_t qPrintVTheta;  
QueueHandle_t qPrintVSumTheta;  
QueueHandle_t qPrintHTheta;  
QueueHandle_t qPrintHSumTheta;  


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


  // Angle read -> motor write
  qMotorAngle = xQueueCreate( 1, sizeof( double ) );
  qMotorSumAngle = xQueueCreate( 1, sizeof( double ) );

  // motor write -> Serial write
  qPrintOutput = xQueueCreate( 1, sizeof( double ) );
  qPrintSetpoint = xQueueCreate( 1, sizeof( double ) );
  qPrintPWMA = xQueueCreate( 1, sizeof( uint8_t ) );
  qPrintPWMB = xQueueCreate( 1, sizeof( uint8_t ) );
  qPrintPWMC = xQueueCreate( 1, sizeof( uint8_t ) );
  qPrintMTheta = xQueueCreate( 1, sizeof( uint8_t ) );
  qPrintMSumTheta = xQueueCreate( 1, sizeof( double ) );
  qPrintVTheta = xQueueCreate( 1, sizeof( uint8_t ) );
  qPrintVSumTheta = xQueueCreate( 1, sizeof( double ) );
  qPrintHTheta = xQueueCreate( 1, sizeof( uint8_t ) );
  qPrintHSumTheta = xQueueCreate( 1, sizeof( double ) );

  // xTaskCreatePinnedToCore(vspiCommand16, "vspi", 4096, (void *)1, 1, NULL, 0);
  // xTaskCreatePinnedToCore(hspiCommand16, "hspi", 4096, (void *)2, 1, NULL, 1);
  xTaskCreatePinnedToCore(vspiCommand8, "vspi", 4096, (void *)1, 1, NULL, 0);

  xTaskCreatePinnedToCore(hspiCommand8, "hspi", 4096, (void *)2, 1, NULL, 1);

  xTaskCreatePinnedToCore(M1_ctrl, "M1_ctrl", 4096, (void *)1, 1, NULL, 1);

  xTaskCreatePinnedToCore(M2_ctrl, "M1_ctrl", 4096, (void *)1, 1, NULL, 1);

  xTaskCreatePinnedToCore(printer, "printer", 4096, (void *)1, 1, NULL, 1);

  Serial.begin(115200);


}

// the loop function runs over and over again until power down or reset
void loop() {
  vTaskDelay(1 / portTICK_RATE_MS);
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
    vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
    digitalWrite(V_CS, LOW);
    uiAngle = vspi->transfer(0x00);
    digitalWrite(V_CS, HIGH);
    theta = (uiAngle*360.0)/65536.0;
    vspi->endTransaction();

    xQueueOverwrite(qMotorAngle, &uiAngle);
    xQueueOverwrite(qPrintVTheta, &uiAngle);

    vTaskDelay(1 / portTICK_RATE_MS);
  }
}

void hspiCommand8(void *pvParameters) {
  uint8_t uiAngle;
  double theta, deltaTheta, sumTheta = 0, prevTheta = 0;

  while(1){
    hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
    digitalWrite(H_CS, LOW);
    uiAngle = hspi->transfer(0x00);
    digitalWrite(H_CS, HIGH);
    theta = (uiAngle*360.0)/65536.0;
    hspi->endTransaction();

    //xQueueOverwrite(qMotorAngle, &uiAngle);
    xQueueOverwrite( qPrintHTheta, &uiAngle);

    vTaskDelay(1 / portTICK_RATE_MS);
  }
}

void M1_ctrl(void *pvParameters) {

  //Angle measurement
  uint8_t theta_raw;
  uint8_t theta_multiplied;
  uint8_t lead_lag;
  uint8_t theta;
  uint8_t theta2;
  uint8_t theta3;

  //Calculation variables
  uint8_t phaseShift8_120 = 256/3;
  uint8_t phaseShift8_90 = 64;
  uint8_t phaseShift8_90_minus = 192;
  uint8_t startPos=0;
  double outputScale;
  
  //Initializatin values;
  int start = 1;
  uint8_t theta_0 = 0;
  double u = 0.2;
  
  //Starting motor in position zero
  uint8_t theta_0_mek = 0;
  uint8_t currentStepA = 0;
  uint8_t currentStepB = currentStepA + 85;
  uint8_t currentStepC = currentStepB + 85;

  //pwm
  uint8_t pwmA, pwmB, pwmC;

  pinMode(M1_PWM1, OUTPUT);
  pinMode(M1_PWM2, OUTPUT);
  pinMode(M1_PWM3, OUTPUT);

  pinMode(M1_EN, OUTPUT);
  pinMode(M1_nRst, OUTPUT);
  pinMode(M1_nSlp, OUTPUT);

  digitalWrite(M1_EN, HIGH);
  digitalWrite(M1_nRst, HIGH);
  digitalWrite(M1_nSlp, HIGH);

    // PWM
  // sigmaDeltaSetup(M1_CH1, PWM_FRQ);
  // sigmaDeltaSetup(M1_CH2, PWM_FRQ);
  // sigmaDeltaSetup(M1_CH3, PWM_FRQ);
  ledcSetup(M1_CH1, PWM_FRQ, PWM_RES);
  ledcSetup(M1_CH2, PWM_FRQ, PWM_RES);
  ledcSetup(M1_CH3, PWM_FRQ, PWM_RES);

  // sigmaDeltaAttachPin(M1_PWM1, M1_CH1);
  // sigmaDeltaAttachPin(M1_PWM2, M1_CH2);
  // sigmaDeltaAttachPin(M1_PWM3, M1_CH3);

  ledcAttachPin(M1_PWM1, M1_CH1);
  ledcAttachPin(M1_PWM2, M1_CH2);
  ledcAttachPin(M1_PWM3, M1_CH3);

  pwmA = (pwmSin[currentStepA]);
  pwmB = (pwmSin[currentStepB]);
  pwmC = (pwmSin[currentStepC]);
  
  // sigmaDeltaWrite(M1_CH1, 0);
  // sigmaDeltaWrite(M1_CH2, 85);
  // sigmaDeltaWrite(M1_CH3, 170);

  ledcWrite(M1_CH1, 0);
  ledcWrite(M1_CH2, 85);
  ledcWrite(M1_CH3, 170);

  vTaskDelay(5000 / portTICK_RATE_MS);
  
  Serial.println("Synchronizing");
  Serial.println("start");
  

  xQueuePeek(qMotorAngle, &theta_0_mek, 0);
  theta_0 = theta_0_mek << 2;
  Serial.println(theta_0);

  while(1){
    for(uint8_t i = 0; i < 256; i+=2)
    {
      
      pwmA = i;
      pwmB = pwmA + 85;
      pwmC = pwmB + 85;
      ledcWrite(M1_CH1, pwmSin[pwmA]);  
      ledcWrite(M1_CH2, pwmSin[pwmB]);
      ledcWrite(M1_CH3, pwmSin[pwmC]); 

      // Serial.print(" A :");
      // Serial.print(pwmSin[pwmA]);
      // Serial.print(" | B : ");
      // Serial.print(pwmSin[pwmB]);
      // Serial.print(" | C : ");
      // Serial.println(pwmSin[pwmC]); 
      usleep(10);
      esp_task_wdt_reset();
    }
    
    
  }


  while(1){
    //Leading or lagging electrical field
    if (u <= 0) lead_lag = phaseShift8_90;
    else if (u > 0) lead_lag = phaseShift8_90_minus;
    
    // Read angle
    xQueuePeek(qMotorAngle, &theta_raw, 0);
    theta_multiplied = theta_raw << 2;        //Multiplied by 4 for electrical angle.
    startPos = 255 - theta_0 + 1;  
    theta_multiplied += startPos;
    theta_multiplied += lead_lag;             //Adding lead/lag angle based on direction
    currentStepA = theta_multiplied;          //Masked by conversion back to 16 bit to cycle around 360 deg 4 times in one mechanical rotation. (Modulus 360 degrees)
    currentStepB = currentStepA + phaseShift8_120;
    currentStepC = currentStepB + phaseShift8_120;

    //Output
    // if (Serial.available() > 0) {
    //         // read the incoming byte:
    //   float buf = Serial.parseFloat();
    //   if(buf != 0){
    //     u = buf;
    //   }
    //   if(buf == 's'){
    //     u = 0;
    //   }
    // }

    outputScale = fabs(u);

    //Output
    pwmA = (int)(pwmSin[currentStepA]);
    pwmB = (int)(pwmSin[currentStepB]);
    pwmC = (int)(pwmSin[currentStepC]);
    // sigmaDeltaWrite(M1_CH1, pwmA);
    // sigmaDeltaWrite(M1_CH2, pwmB);
    // sigmaDeltaWrite(M1_CH3, pwmC);
    ledcWrite(M1_CH1, pwmA);
    ledcWrite(M1_CH2, pwmB);
    ledcWrite(M1_CH3, pwmC);
    
    xQueueOverwrite(qPrintPWMA, &pwmA);
    xQueueOverwrite(qPrintPWMB, &pwmB);
    xQueueOverwrite(qPrintPWMC, &pwmC);

    xQueueOverwrite(qPrintMTheta, &theta_raw);

    xQueueOverwrite(qPrintVTheta, &theta_multiplied);

    vTaskDelay(1 / portTICK_RATE_MS);
    }
}

void M2_ctrl(void *pvParameters) {
  uint8_t pwmA, pwmB, pwmC;

  pinMode(M2_PWM1, OUTPUT);
  pinMode(M2_PWM2, OUTPUT);
  pinMode(M2_PWM3, OUTPUT);

  pinMode(M2_EN, OUTPUT);
  pinMode(M2_nRst, OUTPUT);
  pinMode(M2_nSlp, OUTPUT);

  digitalWrite(M2_EN, HIGH);
  digitalWrite(M2_nRst, HIGH);
  digitalWrite(M2_nSlp, HIGH);

  //PWM
  ledcSetup(M2_CH1, PWM_FRQ, PWM_RES);
  ledcSetup(M2_CH2, PWM_FRQ, PWM_RES);
  ledcSetup(M2_CH3, PWM_FRQ, PWM_RES);

  ledcAttachPin(M2_PWM1, M2_CH1);
  ledcAttachPin(M2_PWM2, M2_CH2);
  ledcAttachPin(M2_PWM3, M2_CH3);

  ledcWrite(M2_CH1, 0);
  ledcWrite(M2_CH2, 85);
  ledcWrite(M2_CH3, 170);

  vTaskDelay(5000 / portTICK_RATE_MS);
  
  Serial.println("Synchronizing");
  Serial.println("start");

  while(1){
    for(uint8_t i = 0; i < 256; i+=2)
    {
      
      pwmA = i;
      pwmB = pwmA + 85;
      pwmC = pwmB + 85;
      ledcWrite(M2_CH1, pwmSin[pwmA]);  
      ledcWrite(M2_CH2, pwmSin[pwmB]);
      ledcWrite(M2_CH3, pwmSin[pwmC]); 

      // Serial.print(" A :");
      // Serial.print(pwmSin[pwmA]);
      // Serial.print(" | B : ");
      // Serial.print(pwmSin[pwmB]);
      // Serial.print(" | C : ");
      // Serial.println(pwmSin[pwmC]); 
      usleep(10);
      esp_task_wdt_reset();
    }
}

}

void printer(void *pvParameters) {
  double output, Setpoint, vSumTheta, hSumTheta, moSumTheta;
  uint8_t hTheta, vTheta, moTheta;
  uint8_t pwmA, pwmB, pwmC; 

  while(1){
    xQueuePeek( qPrintOutput, &output, 0 );
    xQueuePeek( qPrintSetpoint, &Setpoint, 0 );
    xQueuePeek( qPrintMTheta, &moTheta, 0 );
    xQueuePeek( qPrintMSumTheta, &moSumTheta, 0 );
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
    Serial.print(" | vSumTheta: ");
    Serial.print(vSumTheta);
    Serial.print(" | hTheta: ");
    Serial.print(hTheta);
    Serial.print(" | hSumTheta: ");
    Serial.print(hSumTheta);
    Serial.print(" | mTheta: ");
    Serial.println(moTheta);
    // Serial.print(" | PWM : ");
    // Serial.print(pwmA);
    // Serial.print(" | ");
    // Serial.print(pwmB);
    // Serial.print(" | ");
    // Serial.println(pwmC);

    vTaskDelay(100 / portTICK_RATE_MS);
  }
  
}