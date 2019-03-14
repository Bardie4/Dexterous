#include <SPI.h>
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include <PID_v1.h>

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#define M1_PWM1 0
#define M1_PWM2 2
#define M1_PWM3 4
#define M1_EN   16
#define PWM_FRQ 250000

#define H_MOSI  13
#define H_MISO  12
#define H_CLK   14
#define H_CS    15

#define V_MOSI  23
#define V_MISO  19
#define V_CLK   18
#define V_CS    5

#define UART_BAUDRATE       115200        //UART data rate in bits per second (baud)
#define SPI_SCLK_FREQUENCY  1000000      //SPI SCLK Clock frequency in Hz
#define SPI_CS_PIN          10            //SPI CS pin

static const int spiClk = 10*1000*1000; // 10 MHz

//uninitalised pointers to SPI objects
SPIClass * vspi = NULL;
SPIClass * hspi = NULL;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
double probe;
unsigned long time_old;
unsigned long time_new;
unsigned long time_loop;

//Angle measurement
uint8_t theta_raw;
uint8_t theta_multiplied;
uint8_t theta_0;
uint8_t lead_lag;
uint8_t theta;
uint8_t theta2;
uint8_t theta3;

//Specify the links and initial tuning parameters
double Kp = 0.004, Ki = 0, Kd = 0.00;
double consKp = Kp, consKi=Ki, consKd=Kd;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// SPWM (Sine Wave)
const int pwmSin1[] = {128,133,138,144,149,155,160,165,171,176,181,186,192,197,202,207,212,217,222,227,232,236,239,241,242,243,245,246,247,248,249,250,251,252,252,253,253,254,254,255,255,255,255,255,255,255,255,254,254,254,253,252,252,251,250,249,248,247,246,245,244,242,241,240,238,240,241,242,244,245,246,247,248,249,250,251,252,252,253,254,254,254,255,255,255,255,255,255,255,255,254,254,253,253,252,252,251,250,249,248,247,246,245,243,242,241,239,236,232,227,222,217,212,207,202,197,192,186,181,176,171,165,160,155,149,144,138,133,128,122,117,111,106,100,95,90,84,79,74,69,63,58,53,48,43,38,33,28,23,19,16,14,13,12,10,9,8,7,6,5,4,3,3,2,2,1,1,0,0,0,0,0,0,0,0,1,1,1,2,3,3,4,5,6,7,8,9,10,11,13,14,15,17,15,14,13,11,10,9,8,7,6,5,4,3,3,2,1,1,1,0,0,0,0,0,0,0,0,1,1,2,2,3,3,4,5,6,7,8,9,10,12,13,14,16,19,23,28,33,38,43,48,53,58,63,69,74,79,84,90,95,100,106,111,117,122};
 
/// SVPWM (Space Vector Wave)
const int pwmSin[] = {128,131,134,137,140,143,146,149,152,155,158,162,165,167,170,173,176,179,182,185,188,190,193,196,198,201,203,206,208,211,213,215,218,220,222,224,226,228,230,232,234,235,237,238,240,241,243,244,245,246,248,249,250,250,251,252,253,253,254,254,254,255,255,255,255,255,255,255,254,254,254,253,253,252,251,250,250,249,248,246,245,244,243,241,240,238,237,235,234,232,230,228,226,224,222,220,218,215,213,211,208,206,203,201,198,196,193,190,188,185,182,179,176,173,170,167,165,162,158,155,152,149,146,143,140,137,134,131,128,124,121,118,115,112,109,106,103,100,97,93,90,88,85,82,79,76,73,70,67,65,62,59,57,54,52,49,47,44,42,40,37,35,33,31,29,27,25,23,21,20,18,17,15,14,12,11,10,9,7,6,5,5,4,3,2,2,1,1,1,0,0,0,0,0,0,0,1,1,1,2,2,3,4,5,5,6,7,9,10,11,12,14,15,17,18,20,21,23,25,27,29,31,33,35,37,40,42,44,47,49,52,54,57,59,62,65,67,70,73,76,79,82,85,88,90,93,97,100,103,106,109,112,115,118,121,124};
uint8_t currentStepA;
uint8_t currentStepB;
uint8_t currentStepC;
int sineArraySize;

void setup() {
  // SPI
  vspi = new SPIClass(VSPI);
  hspi = new SPIClass(HSPI);
  //SCLK = 18, MISO = 19, MOSI = 23, SS = 5
  vspi->begin(V_CLK, V_MISO, V_MOSI, V_CS);
  //SCLK = 14, MISO = 12, MOSI = 13, SS = 15
  hspi->begin(H_CLK, H_MISO, H_MOSI, H_CS); 
  pinMode(V_CS, OUTPUT); //VSPI CS
  pinMode(H_CS, OUTPUT); //HSPI CS

  // MOTOR
  pinMode(M1_PWM1, OUTPUT); 
  pinMode(M1_PWM2, OUTPUT); 
  pinMode(M1_PWM3, OUTPUT); 
  pinMode(M1_EN, OUTPUT); 
 
  digitalWrite(M1_EN, HIGH);

  // PWM
  sigmaDeltaSetup(0, PWM_FRQ);
  sigmaDeltaSetup(1, PWM_FRQ);
  sigmaDeltaSetup(2, PWM_FRQ);

  sigmaDeltaAttachPin(M1_PWM1, 0);
  sigmaDeltaAttachPin(M1_PWM2, 1);
  sigmaDeltaAttachPin(M1_PWM3, 2);

  // SERIAL
  Serial.begin(UART_BAUDRATE);
  
  // SINE
  sineArraySize = sizeof(pwmSin)/sizeof(int); // Find lookup table size
  sineArraySize--; // Convert from array Size to last PWM array number
}

void loop() {
  //Calculation variables
  uint8_t phaseShift8_120=85;
  uint8_t phaseShift8_90=64;
  uint8_t phaseShift8_90_minus=192;
  uint8_t startPos=0;
  
  //Initializatin values;
  int start = 1;
  uint8_t theta_0 = 0;
  double u = 0;
  
  //Starting motor in position zero
  uint8_t theta_0_mek = 0;
  uint8_t currentStepA = 0;
  uint8_t currentStepB = currentStepA + 85;
  uint8_t currentStepC = currentStepB + 85;
  sigmaDeltaWrite(M1_PWM1, pwmSin[currentStepA]*0.5);
  sigmaDeltaWrite(M1_PWM2, pwmSin[currentStepB]*0.5);
  sigmaDeltaWrite(M1_PWM3, pwmSin[currentStepC]*0.5);
  
  Serial.println("Synchronizing");
  delay(2000);   //Waiting for motor to settle in position 0
  
  Serial.println("start");
  theta_0_mek=hspiRead();
  theta_0=theta_0_mek << 2;
  while(1){
    
    for (int i=0; i <10000; i++){
      
      //Leading or lagging electrical field
      if (u <= 0) lead_lag = phaseShift8_90;
      else if (u > 0) lead_lag = phaseShift8_90_minus;
      
      // Read angle
      theta_raw = hspiRead();
      theta_multiplied = theta_raw << 2;      //Multiplied by 4 for electrical angle.
      startPos=255 - theta_0 +1;
      theta_multiplied += startPos;
      theta_multiplied += lead_lag;            //Adding lead/lag angle based on direction
      currentStepA = theta_multiplied;      //Masked by conversion back to 16 bit to cycle around 360 deg 4 times in one mechanical rotation. (Modulus 360 degrees)
      currentStepB = currentStepA + phaseShift8_120;
      currentStepC = currentStepB + phaseShift8_120;

      // //Output
      // if (Serial.available() > 0) {
      //         // read the incoming byte:
      //        float buf = Serial.parseFloat();
      //        if(buf != 0){
      //           u = buf;
      //        }
      //         if(buf == 's'){
      //           u = 0;
      //        }
              
      // }

      Output = abs(u);
      //Output
      sigmaDeltaWrite(M1_PWM1, pwmSin[currentStepA]*Output);
      sigmaDeltaWrite(M1_PWM2, pwmSin[currentStepB]*Output);
      sigmaDeltaWrite(M1_PWM3, pwmSin[currentStepC]*Output);

      Serial.print("Output: ");
      Serial.print(Output);
      Serial.print(" | Theta: ");
      Serial.print(theta_raw);
      Serial.print(" | pwm: ");
      Serial.println(pwmSin[currentStepA]);

      delay(10);
    }


//   Serial.print("R: ");
//   Serial.print(Setpoint);
//   Serial.print(" | O: ");
////   Serial.print(u);
//    Serial.print(" | A: ");
//    Serial.print(currentStepA);
//    Serial.print(" | B: ");
//    Serial.print(currentStepB);
//    Serial.print(" | C: ");
//    Serial.print(currentStepC);
//    Serial.print("time: ");
//    Serial.print(time_loop);
//    Serial.print(" | PWM_A: ");
//    Serial.print(pwmSin[currentStepA]*Output);
//    Serial.print(" | PWM_B: ");
//    Serial.print(pwmSin[currentStepB]*Output);
//    Serial.print(" | PWM_C: ");
//    Serial.println(pwmSin[currentStepC]*Output);
//   Serial.print(" | step_A: ");
//   Serial.print(currentStepA);
//    Serial.print(" | size_array: ");
//    Serial.println(lead_lag);
//   time_loop=millis()-time_loop;
//   Serial.print(" | step_C: ");
//   Serial.print(time_loop);
//   Serial.print(" | lead_lag: ");
//   Serial.print(lead_lag);
//   Serial.print(" | e_Theta: ");
//   Serial.println(e_theta);
//    Serial.print("Sum: |");
//    Serial.print(sumTheta);
//    Serial.print("Theta: ");
//    Serial.println(theta);
//    Serial.print("time: ");
//    Serial.println(time2);

  // 47 deg - 130 deg
  }
}

uint8_t vspiRead(){
  uint8_t uiAngle;
  double theta, deltaTheta, sumTheta = 0, prevTheta = 0;

  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
  digitalWrite(V_CS, LOW);
  uiAngle = vspi->transfer(0x00);
  digitalWrite(V_CS, HIGH);
  theta = (uiAngle*360.0)/65536.0;
  vspi->endTransaction();

  return uiAngle;
}

uint8_t hspiRead(){
  uint8_t uiAngle;
  double theta, deltaTheta, sumTheta = 0, prevTheta = 0;

  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
  digitalWrite(V_CS, LOW);
  uiAngle = vspi->transfer(0x00);
  digitalWrite(V_CS, HIGH);
  theta = (uiAngle*360.0)/65536.0;
  vspi->endTransaction();

  return uiAngle;
}