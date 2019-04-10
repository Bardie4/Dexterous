#include <MagAlpha.h>
#include <PID_v1.h>

//Check https://www.arduino.cc/en/reference/SPI for SPI signals connections

#define UART_BAUDRATE       115200        //UART data rate in bits per second (baud)
#define SPI_SCLK_FREQUENCY  10000000      //SPI SCLK Clock frequency in Hz
#define SPI_CS_PIN          8            //SPI CS pin

MagAlpha magAlpha;

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

const int EN1 = 6;
const int EN2 = 7;
const int EN3 = 8;
 
const int IN1 = 3;
const int IN2 = 9;
const int IN3 = 10;
const int fault = 2;
const int nreset = 8;
const int line = 4;
const int line2 = 5;

// SPWM (Sine Wave)
//const int pwmSin1[] = {128,133,138,144,149,155,160,165,171,176,181,186,192,197,202,207,212,217,222,227,232,236,239,241,242,243,245,246,247,248,249,250,251,252,252,253,253,254,254,255,255,255,255,255,255,255,255,254,254,254,253,252,252,251,250,249,248,247,246,245,244,242,241,240,238,240,241,242,244,245,246,247,248,249,250,251,252,252,253,254,254,254,255,255,255,255,255,255,255,255,254,254,253,253,252,252,251,250,249,248,247,246,245,243,242,241,239,236,232,227,222,217,212,207,202,197,192,186,181,176,171,165,160,155,149,144,138,133,128,122,117,111,106,100,95,90,84,79,74,69,63,58,53,48,43,38,33,28,23,19,16,14,13,12,10,9,8,7,6,5,4,3,3,2,2,1,1,0,0,0,0,0,0,0,0,1,1,1,2,3,3,4,5,6,7,8,9,10,11,13,14,15,17,15,14,13,11,10,9,8,7,6,5,4,3,3,2,1,1,1,0,0,0,0,0,0,0,0,1,1,2,2,3,3,4,5,6,7,8,9,10,12,13,14,16,19,23,28,33,38,43,48,53,58,63,69,74,79,84,90,95,100,106,111,117,122};
 
/// SVPWM (Space Vector Wave)
const int pwmSin[] = {128,131,134,137,140,143,146,149,152,155,158,162,165,167,170,173,176,179,182,185,188,190,193,196,198,201,203,206,208,211,213,215,218,220,222,224,226,228,230,232,234,235,237,238,240,241,243,244,245,246,248,249,250,250,251,252,253,253,254,254,254,255,255,255,255,255,255,255,254,254,254,253,253,252,251,250,250,249,248,246,245,244,243,241,240,238,237,235,234,232,230,228,226,224,222,220,218,215,213,211,208,206,203,201,198,196,193,190,188,185,182,179,176,173,170,167,165,162,158,155,152,149,146,143,140,137,134,131,128,124,121,118,115,112,109,106,103,100,97,93,90,88,85,82,79,76,73,70,67,65,62,59,57,54,52,49,47,44,42,40,37,35,33,31,29,27,25,23,21,20,18,17,15,14,12,11,10,9,7,6,5,5,4,3,2,2,1,1,1,0,0,0,0,0,0,0,1,1,1,2,2,3,4,5,5,6,7,9,10,11,12,14,15,17,18,20,21,23,25,27,29,31,33,35,37,40,42,44,47,49,52,54,57,59,62,65,67,70,73,76,79,82,85,88,90,93,97,100,103,106,109,112,115,118,121,124};
uint8_t currentStepA;
uint8_t currentStepB;
uint8_t currentStepC;
int sineArraySize;


void setup() {
  // put your setup code here, to run once:
  //Set the SPI SCLK frequency, SPI Mode and CS pin
  magAlpha.begin(SPI_SCLK_FREQUENCY, MA_SPI_MODE_3, SPI_CS_PIN);
  //Set the Serial Communication used to report the angle
  Serial.begin(UART_BAUDRATE);

  // PID

  //Copy paste
   
 setPwmFrequency(IN1); // Increase PWM frequency to 32 kHz  (make unaudible)
 setPwmFrequency(IN2);
 setPwmFrequency(IN3);


  
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT); 
  pinMode(IN3, OUTPUT); 
  
  pinMode(EN1, OUTPUT); 
  pinMode(EN2, OUTPUT); 
  pinMode(EN3, OUTPUT); 
 
  pinMode(fault, INPUT);
  pinMode(nreset, OUTPUT);
  pinMode(line, OUTPUT);
  pinMode(line2, OUTPUT);
  
  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, HIGH);
  digitalWrite(EN3, HIGH);
  digitalWrite(nreset, HIGH);
  digitalWrite(line, HIGH);
  digitalWrite(line2, HIGH);
 
  sineArraySize = sizeof(pwmSin)/sizeof(int); // Find lookup table size
  sineArraySize--; // Convert from array Size to last PWM array number
}

void loop() {
  //Calculation variables
  uint8_t phaseShift8=85;
  uint8_t phaseShift8_90=64;
  uint8_t phaseShift8_90_minus=192;
  uint8_t startPos=0;
  
  //Initializatin values;
  int start = 1;
  uint8_t theta_0 = 0;
  double u = 0;
  
  //Starting motor in position zero
  uint8_t theta_0_mek =0;
  uint8_t currentStepA = 0;
  uint8_t currentStepB = currentStepA + 85;
  uint8_t currentStepC = currentStepB + 85;

  uint8_t pwmA, pwmB, pwmC;
  
  analogWrite(IN1, pwmSin[currentStepA]*0.5);
  analogWrite(IN2, pwmSin[currentStepB]*0.5);
  analogWrite(IN3, pwmSin[currentStepC]*0.5);
  
  Serial.println("Synchronizing");
  delay(3000);   //Waiting for motor to settle in position 0
  
    Serial.println("start");
  theta_0_mek=magAlpha.readAngleRaw8();
//  theta_0=theta_0_mek << 2;
  while(1){

  
    
   
    
    time_old=micros();
    for (double i=0; i <10000; i++){
      
    //Leading or lagging electrical field
    if (u <= 0) lead_lag = phaseShift8_90_minus;
    else if (u > 0) lead_lag = phaseShift8_90;
    
    // Read angle
    theta_raw = magAlpha.readAngleRaw8();
    theta_raw += (255 - theta_0_mek);
    theta_multiplied =  theta_raw << 2;      //Multiplied by 4 for electrical angle
    theta_multiplied += lead_lag;            //Adding lead/lag angle based on direction
     currentStepA  =  theta_multiplied;    //Masked by conversion back to 16 bit to cycle around 360 deg 4 times in one mechanical rotation. (Modulus 360 degrees)
     currentStepB =   currentStepA + phaseShift8;
     currentStepC =   currentStepB + phaseShift8;

    //Output
    if (Serial.available() > 0) {
            // read the incoming byte:
           float buf = Serial.parseFloat();
           if(buf != 0){
              u = buf;
           }
            if(buf == 's'){
              u = 0;
           }
            
    }
    Output=abs(u);
    //Output
    analogWrite(IN1, pwmSin[currentStepA]*Output);
    analogWrite(IN2, pwmSin[currentStepB]*Output);
    analogWrite(IN3, pwmSin[currentStepC]*Output);

    Serial.println(theta_raw);

//    Serial.print("A: ");
//    Serial.print(currentStepA);
//    Serial.print(" B: ");
//    Serial.print(currentStepB);
//    Serial.print(" C: ");
//    Serial.print(currentStepC);
//    Serial.print(" Theta: ");
//    Serial.println(theta_multiplied);
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
void setPwmFrequency(int pin) {
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | 0x01;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | 0x01;
    }
  }
  else if(pin == 3 || pin == 11) {
    TCCR2B = TCCR2B & 0b11111000 | 0x01;
  }
}
