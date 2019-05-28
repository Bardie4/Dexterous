#include <SPI.h>
//#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include <unistd.h>
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include <PID_v1.h>

#include "i2c_slave.h"

//#include "BluetoothSerial.h" //Header File for Serial Bluetooth, will be added by default into Arduino


#define I2C_DEBUG false
#define BLT_DEBUG false
#define PWM_DEBUG true
#define SPI_DEBUG false

SemaphoreHandle_t xPrintMtx;
SemaphoreHandle_t xMotorHMtx;
SemaphoreHandle_t xMotorVMtx;

#define TASK_RESET_PERIOD_S 10

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

#define M1_PWMU 25
#define M1_PWMV 33
#define M1_PWMW 32
#define M1_nFlt 34
#define M1_CH1  0
#define M1_CH2  1
#define M1_CH3  2

#define M2_PWMU 15
#define M2_PWMV 2 
#define M2_PWMW 4 
#define M2_nFlt 35
#define M2_CH1  3
#define M2_CH2  4
#define M2_CH3  5

#define M_EN    22

#define PWM_FRQ 300000
#define PWM_RES 8

#define I2C_SLAVE_ADDR 0x28

//uninitalised pointers to SPI objects
SPIClass * vspi = NULL;
SPIClass * hspi = NULL;

static const int spiClk = 20*1000*1000; // 20 MHz

// i2c packet
typedef struct{
  uint8_t commandbyte, m1_torque, m2_torque;
} cmd_t;

// Global queuhandles
QueueHandle_t q_angle_V, q_scale_V, q_dir_V, q_angle_H, q_scale_H, q_dir_H;

// Lookup table
const uint8_t pwmSin[] = {128,131,134,137,140,143,146,149,152,155,158,162,165,167,170,173,176,179,182,185,188,190,193,196,198,201,203,206,208,211,213,215,218,220,222,224,226,228,230,232,234,235,237,238,240,241,243,244,245,246,248,249,250,250,251,252,253,253,254,254,254,255,255,255,255,255,255,255,254,254,254,253,253,252,251,250,250,249,248,246,245,244,243,241,240,238,237,235,234,232,230,228,226,224,222,220,218,215,213,211,208,206,203,201,198,196,193,190,188,185,182,179,176,173,170,167,165,162,158,155,152,149,146,143,140,137,134,131,128,124,121,118,115,112,109,106,103,100,97,93,90,88,85,82,79,76,73,70,67,65,62,59,57,54,52,49,47,44,42,40,37,35,33,31,29,27,25,23,21,20,18,17,15,14,12,11,10,9,7,6,5,5,4,3,2,2,1,1,1,0,0,0,0,0,0,0,1,1,1,2,2,3,4,5,5,6,7,9,10,11,12,14,15,17,18,20,21,23,25,27,29,31,33,35,37,40,42,44,47,49,52,54,57,59,62,65,67,70,73,76,79,82,85,88,90,93,97,100,103,106,109,112,115,118,121,124};

struct params
{
  // ID for debugging purposes
  const int id;
  // Motor
  const int PWMU, PWMV, PWMW, NFLT, CHN1, CHN2, CHN3;
  // SPI
  SPIClass* spi_com;
  uint8_t spi_bus;
  const int MOSI, CS, MISO, CLK;
  // Queue
  QueueHandle_t q_angle, q_scale, q_dir;

};

params joint_V =  {
    .id = 0,
    .PWMU = M1_PWMU, .PWMV = M1_PWMV, .PWMW = M1_PWMW, 
    .NFLT = M1_nFlt, .CHN1 = M1_CH1, .CHN2 = M1_CH2, .CHN3 = M1_CH3, 
    .spi_com = hspi, .spi_bus = HSPI,
    .MOSI = H_MOSI, .CS = H_CS, .MISO = H_MISO, .CLK = H_CLK,
    .q_angle = q_angle_H, .q_scale = q_scale_H, .q_dir = q_dir_H

};

params joint_H = {
    .id = 1,
    .PWMU = M2_PWMU, .PWMV = M2_PWMV, .PWMW = M2_PWMW, 
    .NFLT = M2_nFlt, .CHN1 = M2_CH1, .CHN2 = M2_CH2, .CHN3 = M2_CH3, 
    .spi_com = vspi, .spi_bus = VSPI,
    .MOSI = V_MOSI, .CS = V_CS, .MISO = V_MISO, .CLK = V_CLK,
    .q_angle = q_angle_V, .q_scale = q_scale_V, .q_dir = q_dir_V
};

// #######################################################################################
// ########################### JOINT 1 (V) ###############################################
// #######################################################################################

void motorTask(void* pvParameters) {
  params* joint;
  joint = (params *) pvParameters;

  Serial.print(joint->CS);

  uint8_t ui_angle, ui_angle_prev;
  int16_t ui_angle_delta;
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
  uint8_t theta_0_mech = 0;

  // PWM
  uint8_t current_step_U = 0;
  uint8_t current_step_V = current_step_U + 85;
  uint8_t current_step_W = current_step_V + 85;
  uint8_t pwm_U, pwm_V, pwm_W;

  // control scaling
  uint8_t velocity_raw = 0;
  double velocity = 0.0;
  bool dir = true;

  // Initialize motor driver
  pinMode(joint->PWMU, OUTPUT);
  pinMode(joint->PWMV, OUTPUT);
  pinMode(joint->PWMW, OUTPUT);

  pinMode(joint->NFLT, INPUT);

  // PWM setup
  ledcSetup(joint->CHN1, PWM_FRQ, PWM_RES);
  ledcSetup(joint->CHN2, PWM_FRQ, PWM_RES);
  ledcSetup(joint->CHN3, PWM_FRQ, PWM_RES);

  ledcAttachPin(joint->PWMU, joint->CHN1);
  ledcAttachPin(joint->PWMV, joint->CHN2);
  ledcAttachPin(joint->PWMW, joint->CHN3);

  // Initial PWM value
  pwm_U = (pwmSin[current_step_U]);
  pwm_V = (pwmSin[current_step_V]);
  pwm_W = (pwmSin[current_step_W]);

  ledcWrite(joint->CHN1, 0);
  ledcWrite(joint->CHN2, 85);
  ledcWrite(joint->CHN3, 170);

  vTaskDelay(2000 / portTICK_RATE_MS);

  
  xQueuePeek(joint->q_angle, &theta_0_mech, 0);
  Serial.println("theta_0_mech");
  Serial.println(theta_0_mech);
    
  for(;;){    
    // Read angle
    joint->spi_com->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
    digitalWrite(joint->CS, LOW);
    theta_raw = joint->spi_com->transfer(0x00);
    digitalWrite(joint->CS, HIGH);
    //theta = (ui_angle*360.0)/65536.0;
    joint->spi_com->endTransaction();
    
    // Read master commands
    xQueuePeek(joint->q_scale, &velocity_raw, 0);
    xQueuePeek(joint->q_dir,   &dir,          0);
    velocity = (double)((velocity_raw / 255.0) * 1.0);

    //Leading or lagging electrical field
    if (dir) lead_lag = phaseShift8_90;
    else lead_lag = phaseShift8_90_minus;

    // Find optimal electrical angle
    theta_raw += (255 - theta_0_mech);
    theta_multiplied = theta_raw << 2;        //Multiplied by 4 for electrical angle.
    theta_multiplied += lead_lag;             //Adding lead/lag angle based on direction
    current_step_U = theta_multiplied;          //Masked by conversion back to 16 bit to cycle around 360 deg 4 times in one mechanical rotation. (Modulus 360 degrees)
    current_step_V = current_step_U + phaseShift8_120;
    current_step_W = current_step_V + phaseShift8_120;

    //Output
    pwm_U = (uint8_t)((double)pwmSin[current_step_U] * velocity ); //* velocity_scale);
    pwm_V = (uint8_t)((double)pwmSin[current_step_V] * velocity ); //* velocity_scale);
    pwm_W = (uint8_t)((double)pwmSin[current_step_W] * velocity ); //* velocity_scale);

    ledcWrite(joint->CHN1, pwm_U);
    ledcWrite(joint->CHN2, pwm_V);
    ledcWrite(joint->CHN3, pwm_W);
    
    // Feed watchdog
    TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed = 1;
    TIMERG0.wdt_wprotect = 0;

    if (PWM_DEBUG)
    {
      xSemaphoreTake( xPrintMtx, portMAX_DELAY );
      if (joint->id == 1)
      {
        Serial.print("                                                      ");
      }
      Serial.print(" |Theta: ");
      Serial.print(theta_raw);
      // Serial.print(" | Theta_mult: ");
      // Serial.print(theta_multiplied);
      Serial.print(" | dir: ");
      Serial.print(dir);
      Serial.print(" | vel: ");
      Serial.print(velocity);
      Serial.print(" | PWM: ");
      Serial.print(pwm_U);
      Serial.print(" ");
      Serial.print(pwm_V);
      Serial.print(" ");
      Serial.println(pwm_W);
      xSemaphoreGive( xPrintMtx );
    }
  }
}

// the loop function runs over and over again until power down or reset
void getAngleTask(void *pvParameters) {
  params *joint;
  joint = (params *) pvParameters;

  uint8_t ui_angle;

  for (;;)
  {
    joint->spi_com->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
    digitalWrite(joint->CS, LOW);
    ui_angle = joint->spi_com->transfer(0x00);
    digitalWrite(joint->CS, HIGH);
    //theta = (ui_angle*360.0)/65536.0;
    joint->spi_com->endTransaction();

    xQueueOverwrite(joint->q_angle, &ui_angle);

    if (SPI_DEBUG)
    {
      xSemaphoreTake( xPrintMtx, portMAX_DELAY );
      Serial.print(" Node: ");
      Serial.print(joint->id);
      Serial.print(" | ui_angle: ");
      Serial.println(ui_angle);
      xSemaphoreGive( xPrintMtx );
    }
    
    // TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
    // TIMERG0.wdt_feed = 1;
    // TIMERG0.wdt_wprotect = 0;
    vTaskDelay(1 / portTICK_RATE_MS);
  }
}

// // #######################################################################################
// // ########################### MASTER COM ################################################
// // #######################################################################################

void masterComTask(void *pvParameters) {
  i2c_slave i2c_s(I2C_SLAVE_ADDR);
  i2c_packet i2c_received;

  cmd_t command;
  bool directionV, directionH;
  uint8_t commandbyte, m1_torque, m2_torque;

  uint8_t send_buf[3];

  while (1)
  {
    i2c_received = i2c_s.read();

    if (i2c_received.size){
      //i2c_reset_tx_fifo(I2C_NUM_0);
      // Read three latest
      commandbyte = i2c_received.data[i2c_received.size - 3];
      m1_torque   = i2c_received.data[i2c_received.size - 2];
      m2_torque   = i2c_received.data[i2c_received.size - 1];

      // bitmask
      directionV = !((commandbyte & ( 1 << 0 )) >> 0); // LSB. Flipped
      directionH = !((commandbyte & ( 1 << 1 )) >> 1); // LSB - 1. Flipped

      send_buf[0] = commandbyte;
      send_buf[1] = m1_torque;
      send_buf[2] = m2_torque;
      //i2c_s.write(buf);

      xQueueOverwrite(joint_V.q_scale, &m1_torque);
      xQueueOverwrite(joint_V.q_dir,   &directionV);
      xQueueOverwrite(joint_H.q_scale, &m2_torque);
      xQueueOverwrite(joint_H.q_dir,   &directionH);

      if (I2C_DEBUG)
      {
        xSemaphoreTake(xPrintMtx, 0);
        Serial.print("I2C: "); Serial.print(commandbyte);
        Serial.print(" "); Serial.print(m1_torque);
        Serial.print(" "); Serial.print(m2_torque);
        Serial.println();
        xSemaphoreGive(xPrintMtx);
      }

      TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
      TIMERG0.wdt_feed = 1;
      TIMERG0.wdt_wprotect = 0;
    }

    TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed = 1;
    TIMERG0.wdt_wprotect = 0;
    vTaskDelay(10 / portTICK_RATE_MS);
  }
}

// void blComTask(void *pvParameters) {
//   BluetoothSerial BT; //Object for Bluetooth
//   char buf;

//   struct rec_buf
//   {
//     char buf;
//     char command;
//     char motor_V;
//     char motor_H;
//   } received;
  
//   bool directionL, directionH;

//   BT.begin("DEX"); // Bluetooth device
//   Serial.println("DEX is Ready to Pair");

//   while (1){
//     if (BT.available()){ //Check if we receive anything from Bluetooth
//       received.buf = BT.read(); //Read what we recevive 
//       Serial.print("Received:"); Serial.println(received.buf);
//       BT.write(received.buf);
//       switch (received.buf){
//         case 'a':
//           received.command = BT.read();
//           break;
//         case 'b':
//           received.motor_V = BT.read();
//           break;
//         case 'c':
//           received.motor_H = BT.read();
//           break;
//         default:
//           break;
//       }
//     }

//     if (BLT_DEBUG)
//     {
//       Serial.print("commandbyte"); Serial.println(received.command);
//       Serial.print("motor_V"); Serial.println((int)received.motor_V);
//       Serial.print("motor_H"); Serial.println((int)received.motor_H);
//     }
    
//     vTaskDelay(10 / portTICK_RATE_MS);
//   }
// }

// #######################################################################################
// ########################### SETUP & LOOP ##############################################
// #######################################################################################

void setup() {

  joint_V.spi_com = new SPIClass(joint_V.spi_bus);
  joint_V.spi_com->begin(joint_V.CLK, joint_V.MISO, joint_V.MOSI, joint_V.CS);
  pinMode(joint_V.CS, OUTPUT);
  joint_V.q_angle = xQueueCreate( 1, sizeof( uint8_t ) );
  joint_V.q_scale = xQueueCreate( 1, sizeof( uint8_t ) );
  joint_V.q_dir   = xQueueCreate( 1, sizeof( bool ) );

  joint_H.spi_com = new SPIClass(joint_H.spi_bus);
  joint_H.spi_com->begin(joint_H.CLK, joint_H.MISO, joint_H.MOSI, joint_H.CS);
  pinMode(joint_H.CS, OUTPUT);
  joint_H.q_angle = xQueueCreate( 1, sizeof( uint8_t ) );
  joint_H.q_scale = xQueueCreate( 1, sizeof( uint8_t ) );
  joint_H.q_dir   = xQueueCreate( 1, sizeof( bool ) );

  joint_V.spi_com->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(joint_V.CS, LOW);
  uint8_t theta_raw = joint_V.spi_com->transfer(0b10000000);
  theta_raw = joint_V.spi_com->transfer(0b00000000);
  usleep(20000);
  uint16_t a = joint_V.spi_com->transfer16(0x0000);
  digitalWrite(joint_V.CS, HIGH);
  joint_V.spi_com->endTransaction();


  pinMode(M1_PWMU, OUTPUT);
  pinMode(M1_PWMV, OUTPUT);
  pinMode(M1_PWMW, OUTPUT);
  pinMode(M2_PWMU, OUTPUT);
  pinMode(M2_PWMV, OUTPUT);
  pinMode(M2_PWMW, OUTPUT);
  digitalWrite(M1_PWMU, LOW);
  digitalWrite(M1_PWMV, LOW);
  digitalWrite(M1_PWMW, LOW);
  digitalWrite(M2_PWMU, LOW);
  digitalWrite(M2_PWMV, LOW);
  digitalWrite(M2_PWMW, LOW);

  pinMode(M_EN, OUTPUT);
  digitalWrite(M_EN, HIGH);

  xPrintMtx = xSemaphoreCreateMutex();
  xMotorHMtx = xSemaphoreCreateMutex();
  xMotorVMtx = xSemaphoreCreateMutex();

  // motor control tasks
  xTaskCreatePinnedToCore(motorTask, "motorV", 4096 * 3, (void *) &joint_V, 1, NULL, 0);  // Link 2
  //xTaskCreatePinnedToCore(motorTask, "motorH", 4096 * 3, (void *) &joint_H, 1, NULL, 0); // Link 1

  // xTaskCreatePinnedToCore(motorScroll, "motorV", 4096, (void *) &joint_V, 1, NULL, 0);
  // xTaskCreatePinnedToCore(motorScroll, "motorH", 4096, (void *) &joint_H, 1, NULL, 0);

  //xTaskCreatePinnedToCore(blComTask, "motorV", 4096, (void *) &joint_V, 1, NULL, 0);
  xTaskCreatePinnedToCore(masterComTask, "com", 4096, (void *) &joint_H, 1, NULL, 1);

  Serial.begin(115200);

    Serial.print("a: ");
  Serial.println(a);
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}

void motorScroll(void *pvParameters) {
  params* joint;
  joint = (params *) pvParameters;

  uint8_t phaseShift8_120 = 256/3;
  uint8_t currentStepA = 0;
  uint8_t currentStepB = currentStepA + 85;
  uint8_t currentStepC = currentStepB + 85;
  //pwm
  uint8_t pwmA, pwmB, pwmC;

  // Initialize motor driver
  pinMode(joint->PWMU, OUTPUT);
  pinMode(joint->PWMV, OUTPUT);
  pinMode(joint->PWMW, OUTPUT);

  pinMode(joint->NFLT, INPUT);

  // PWM setup
  ledcSetup(joint->CHN1, PWM_FRQ, PWM_RES);
  ledcSetup(joint->CHN2, PWM_FRQ, PWM_RES);
  ledcSetup(joint->CHN3, PWM_FRQ, PWM_RES);

  ledcAttachPin(joint->PWMU, joint->CHN1);
  ledcAttachPin(joint->PWMV, joint->CHN2);
  ledcAttachPin(joint->PWMW, joint->CHN3);

  while(1){
    // Scroll through field
    for(uint8_t i = 0; i < 0; i++)
    {
      currentStepA = i; currentStepB = i, currentStepC = i;         //Masked by conversion back to 16 bit to cycle around 360 deg 4 times in one mechanical rotation. (Modulus 360 degrees)
      // currentStepB = currentStepA + phaseShift8_120;
      // currentStepC = currentStepB + phaseShift8_120;
      pwmA = (uint8_t)(pwmSin[currentStepA]);
      pwmB = (uint8_t)(pwmSin[currentStepB]);
      pwmC = (uint8_t)(pwmSin[currentStepC]);
      ledcWrite(joint->CHN1, pwmA);
      ledcWrite(joint->CHN2, pwmB);
      ledcWrite(joint->CHN3, pwmC);
  
      // xQueueOverwrite(qPrintPWM2A, &pwmA);
      // xQueueOverwrite(qPrintPWM2B, &pwmB);
      // xQueueOverwrite(qPrintPWM2C, &pwmC);
      vTaskDelay(10/portTICK_RATE_MS);
    }
  }
}
