#include <SPI.h>
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include <unistd.h>
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include <PID_v1.h>

#include "i2c_slave.h"


#define TASK_RESET_PERIOD_S     10

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

#define M1_PWMU 25 //2
#define M1_PWMV 33 //4
#define M1_PWMW 32 //21
#define M1_nFlt 34
#define M1_CH1  0
#define M1_CH2  1
#define M1_CH3  2

#define M2_PWMU 15 //2
#define M2_PWMV 2 //4
#define M2_PWMW 4 //21
#define M2_nFlt 35
#define M2_CH1  3
#define M2_CH2  4
#define M2_CH3  5

#define M_EN    22

#define PWM_FRQ 300000
#define PWM_RES 8

#define I2C_SLAVE_ADDR 0x28

bool I2C_DEBUG = true;
bool PWM_DEBUG = false;

//uninitalised pointers to SPI objects
SPIClass * vspi = NULL;
SPIClass * hspi = NULL;

static const int spiClk = 20*1000*1000; // 20 MHz

// i2c packet
typedef struct{
  uint8_t commandbyte, m1_torque, m2_torque;
} cmd_t;

// Global queuhandles
QueueHandle_t qScaleL, qDirL, qScaleH, qDirH;

// Lookup table
const uint8_t pwmSin[] = {128,131,134,137,140,143,146,149,152,155,158,162,165,167,170,173,176,179,182,185,188,190,193,196,198,201,203,206,208,211,213,215,218,220,222,224,226,228,230,232,234,235,237,238,240,241,243,244,245,246,248,249,250,250,251,252,253,253,254,254,254,255,255,255,255,255,255,255,254,254,254,253,253,252,251,250,250,249,248,246,245,244,243,241,240,238,237,235,234,232,230,228,226,224,222,220,218,215,213,211,208,206,203,201,198,196,193,190,188,185,182,179,176,173,170,167,165,162,158,155,152,149,146,143,140,137,134,131,128,124,121,118,115,112,109,106,103,100,97,93,90,88,85,82,79,76,73,70,67,65,62,59,57,54,52,49,47,44,42,40,37,35,33,31,29,27,25,23,21,20,18,17,15,14,12,11,10,9,7,6,5,5,4,3,2,2,1,1,1,0,0,0,0,0,0,0,1,1,1,2,2,3,4,5,5,6,7,9,10,11,12,14,15,17,18,20,21,23,25,27,29,31,33,35,37,40,42,44,47,49,52,54,57,59,62,65,67,70,73,76,79,82,85,88,90,93,97,100,103,106,109,112,115,118,121,124};

void setup() {
  vspi = new SPIClass(VSPI);
  hspi = new SPIClass(HSPI);

  vspi->begin(V_CLK, V_MISO, V_MOSI, V_CS);
  hspi->begin(H_CLK, H_MISO, H_MOSI, H_CS); 

  pinMode(V_CS, OUTPUT); //VSPI SS
  pinMode(H_CS, OUTPUT); //HSPI SS

  pinMode(M_EN, OUTPUT);
  digitalWrite(M_EN, HIGH);

  qScaleL = xQueueCreate( 1, sizeof( uint8_t ) );
  qDirL = xQueueCreate( 1, sizeof( bool ) );
  qScaleH = xQueueCreate( 1, sizeof( uint8_t ) );
  qDirH = xQueueCreate( 1, sizeof( bool ) );

  xTaskCreatePinnedToCore(motorLTask, "motorL", 4096, (void *)1, 1, NULL, 0);
  xTaskCreatePinnedToCore(masterCom, "getscaleL", 4096, (void *)1, 1, NULL, 0);
  //xTaskCreatePinnedToCore(motorH, "hspi", 4096, (void *)2, 1, NULL, 1);

  Serial.begin(115200);
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}

void motorLTask(void *pvParameters) {
  uint8_t uiAngle, uiAnglePrev;
  int16_t uiAngleDelta;
  double theta; // deltaTheta, sumTheta = 0, prevTheta = 0;
  double Input;

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
  uint8_t currentStepU = 0;
  uint8_t currentStepV = currentStepU + 85;
  uint8_t currentStepW = currentStepV + 85;
  uint8_t pwmU, pwmV, pwmW;

  // control scaling
  double scaling;

  // Initialize motor driver
  pinMode(M1_PWMU, OUTPUT);
  pinMode(M1_PWMV, OUTPUT);
  pinMode(M1_PWMW, OUTPUT);

  pinMode(M1_nFlt, INPUT);

  // PWM setup
  ledcSetup(M1_CH1, PWM_FRQ, PWM_RES);
  ledcSetup(M1_CH2, PWM_FRQ, PWM_RES);
  ledcSetup(M1_CH3, PWM_FRQ, PWM_RES);

  ledcAttachPin(M1_PWMU, M1_CH1);
  ledcAttachPin(M1_PWMV, M1_CH2);
  ledcAttachPin(M1_PWMW, M1_CH3);

  // Initial PWM value
  pwmU = (pwmSin[currentStepU]);
  pwmV = (pwmSin[currentStepV]);
  pwmW = (pwmSin[currentStepW]);

  ledcWrite(M1_CH1, 0);
  ledcWrite(M1_CH2, 85);
  ledcWrite(M1_CH3, 170);

  vTaskDelay(2000 / portTICK_RATE_MS);

  Serial.println("start M1");
  theta_0_mek = readMagnetL();
    
  while(1){    
    // Read angle
    theta_raw = readMagnetL();
    //xQueuePeek(qScale, &scaling);

    //Leading or lagging electrical field
    if (scaling <= 0) lead_lag = phaseShift8_90;
    else lead_lag = phaseShift8_90_minus;

    scaling/=100.0;

    // Find optimal electrical angle
    theta_raw += (255 - theta_0_mek);
    theta_multiplied = theta_raw << 2;        //Multiplied by 4 for electrical angle.
    theta_multiplied += lead_lag;             //Adding lead/lag angle based on direction
    currentStepU = theta_multiplied;          //Masked by conversion back to 16 bit to cycle around 360 deg 4 times in one mechanical rotation. (Modulus 360 degrees)
    currentStepV = currentStepU + phaseShift8_120;
    currentStepW = currentStepV + phaseShift8_120;

    //Output
    pwmU = (int)(pwmSin[currentStepU] * scaling);
    pwmV = (int)(pwmSin[currentStepV] * scaling);
    pwmW = (int)(pwmSin[currentStepW] * scaling);

    ledcWrite(M1_CH1, pwmU);
    ledcWrite(M1_CH2, pwmV);
    ledcWrite(M1_CH3, pwmW);
    
    TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed = 1;
    TIMERG0.wdt_wprotect = 0;

    if (PWM_DEBUG)
    {
      Serial.print("Theta: ");
      Serial.print(theta_raw);
      Serial.print(" | Theta_mult: ");
      Serial.print(theta_multiplied);
      Serial.print("PWM: ");
      Serial.print(pwmU);
      Serial.print(" ");
      Serial.print(pwmV);
      Serial.print(" ");
      Serial.print(pwmW);
    }
  }
}

// the loop function runs over and over again until power down or reset
uint8_t readMagnetL() {
  uint8_t uiAngle;

  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(V_CS, LOW);
  uiAngle = vspi->transfer(0x00);
  digitalWrite(V_CS, HIGH);
  //theta = (uiAngle*360.0)/65536.0;
  vspi->endTransaction();

  return uiAngle;
}

void masterCom(void *pvParameters) {
  i2c_slave i2c_s(I2C_SLAVE_ADDR);
  i2c_packet i2c_received;

  cmd_t command;
  bool directionL, directionH;

  while (1)
  {
    i2c_received = i2c_s.read();

    if (i2c_received.size){
      // for (int i = i2c_received.size - 3; i < i2c_received.size; i++){ // Latest values
      //   Serial.print(i);
      //   Serial.print(": ");
      //   Serial.print(i2c_received.data[i]);
      //   Serial.print(" ");
      // }

      // Read three latest
      command.commandbyte = i2c_received.data[i2c_received.size - 3];
      command.m1_torque = i2c_received.data[i2c_received.size - 2];
      command.m2_torque = i2c_received.data[i2c_received.size - 1];

      // bitmask
      directionL = (command.commandbyte & ( 1 << 0 )) >> 0; // LSB
      directionH = (command.commandbyte & ( 1 << 1 )) >> 1; // LSB - 1

      uint8_t buf[] = {command.commandbyte, command.m1_torque, command.m2_torque};
      i2c_s.write(command.commandbyte);

      xQueueOverwrite(qScaleL, &command.m1_torque);
      xQueueOverwrite(qDirL, &directionL);
      xQueueOverwrite(qScaleH, &command.m2_torque);
      xQueueOverwrite(qDirH, &directionH);

      if (I2C_DEBUG)
      {
        Serial.print("I2C: ");
        Serial.print(command.commandbyte);
        Serial.print(" ");
        Serial.print(command.m1_torque);
        Serial.print(" ");
        Serial.print(command.m2_torque);
        Serial.println();
      }
    }
    vTaskDelay(100 / portTICK_RATE_MS);
  }
}