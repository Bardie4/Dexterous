#include <MagAlpha.h>
#include <PID_v1.h>

//Check https://www.arduino.cc/en/reference/SPI for SPI signals connections

#define UART_BAUDRATE       115200        //UART data rate in bits per second (baud)
#define SPI_SCLK_FREQUENCY  10000000      //SPI SCLK Clock frequency in Hz
#define SPI_CS_PIN          10            //SPI CS pin

MagAlpha magAlpha;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp = 1.5, Ki = 0, Kd = 0.00;
double consKp = Kp, consKi=Ki, consKd=Kd;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

const int EN1 = 5;
const int EN2 = 6;
const int EN3 = 7;
 
const int IN1 = 9;
const int IN2 = 10;
const int IN3 = 11;
 
 
// SPWM (Sine Wave)
//const int pwmSin[] = {127, 138, 149, 160, 170, 181, 191, 200, 209, 217, 224, 231, 237, 242, 246, 250, 252, 254, 254, 254, 252, 250, 246, 242, 237, 231, 224, 217, 209, 200, 191, 181, 170, 160, 149, 138, 127, 116, 105, 94, 84, 73, 64, 54, 45, 37, 30, 23, 17, 12, 8, 4, 2, 0, 0, 0, 2, 4, 8, 12, 17, 23, 30, 37, 45, 54, 64, 73, 84, 94, 105, 116 };
 
 
/// SVPWM (Space Vector Wave)
//const int pwmSin[] = {128, 147, 166, 185, 203, 221, 238, 243, 248, 251, 253, 255, 255, 255, 253, 251, 248, 243, 238, 243, 248, 251, 253, 255, 255, 255, 253, 251, 248, 243, 238, 221, 203, 185, 166, 147, 128, 109, 90, 71, 53, 35, 18, 13, 8, 5, 3, 1, 1, 1, 3, 5, 8, 13, 18, 13, 8, 5, 3, 1, 1, 1, 3, 5, 8, 13, 18, 35, 53, 71, 90, 109};
const int pwmSin[] = {128, 132, 136, 140, 143, 147, 151, 155, 159, 162, 166, 170, 174, 178, 181, 185, 189, 192, 196, 200, 203, 207, 211, 214, 218, 221, 225, 228, 232, 235, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 235, 232, 228, 225, 221, 218, 214, 211, 207, 203, 200, 196, 192, 189, 185, 181, 178, 174, 170, 166, 162, 159, 155, 151, 147, 143, 140, 136, 132, 128, 124, 120, 116, 113, 109, 105, 101, 97, 94, 90, 86, 82, 78, 75, 71, 67, 64, 60, 56, 53, 49, 45, 42, 38, 35, 31, 28, 24, 21, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21, 24, 28, 31, 35, 38, 42, 45, 49, 53, 56, 60, 64, 67, 71, 75, 78, 82, 86, 90, 94, 97, 101, 105, 109, 113, 116, 120, 124};
 
int currentStepA;
int currentStepB;
int currentStepC;
int sineArraySize;
int increment = 0;
double e_theta; //electrical angle
double a_theta; //array angle
boolean direct = 1; // direction true=forward, false=backward

//90 degrees angle;
int ea_nintey = sineArraySize/4;

void setup() {
  // put your setup code here, to run once:
  //Set the SPI SCLK frequency, SPI Mode and CS pin
  magAlpha.begin(SPI_SCLK_FREQUENCY, MA_SPI_MODE_3, SPI_CS_PIN);
  //Set the Serial Communication used to report the angle
  Serial.begin(UART_BAUDRATE);

  // PID
  Input = analogRead(0);
  Setpoint = 70;
  myPID.SetOutputLimits(-1,1);
  myPID.SetMode(AUTOMATIC);

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
 
 
  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, HIGH);
  digitalWrite(EN3, HIGH);
  
 
  sineArraySize = sizeof(pwmSin)/sizeof(int); // Find lookup table size
  int phaseShift = sineArraySize / 3;         // Find phase shift and initial A, B C phase values
  currentStepA = 0;
  currentStepB = currentStepA + phaseShift;
  currentStepC = currentStepB + phaseShift;
 
  sineArraySize--; // Convert from array Size to last PWM array number
}

void loop() {
  double theta, deltaTheta, sumTheta = 0, thetaPrev = 0;
  // double time1, time2;
  double u;
  double g;
  double mg_kt=14;
  analogWrite(IN1, pwmSin[currentStepA]);
  analogWrite(IN2, pwmSin[currentStepB]);
  analogWrite(IN3, pwmSin[currentStepC]);
  delay(3000)
  start=1;
  e_theta_0=0;

  while(1){

    // Read angle
    double theta = magAlpha.readAngle() - 34.0;
    // Angle change
    double deltaTheta = theta - thetaPrev;

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
    thetaPrev = theta;

    // PID
    Input = sumTheta;

    e_theta=4.0*theta-e_theta_0;
    if (e_theta > 360 && e_theta <= 720) e_theta= e_theta - 360;
    else if (e_theta > 720 && e_theta <= 1080) e_theta= e_theta - 720;
    else if (e_theta > 1080 && e_theta <= 1440)e_theta= e_theta - 1080;

    if (start==1){
      e_theta_0=e_theta;
      start=0;
      e_theta=0;
    }

    
    a_theta=e_theta/sineArraySize;
    

    double gap = abs(Setpoint-Input); //distance away from setpoint
    if (gap < 7)
    {  //we're close to setpoint, use conservative tuning parameters
      myPID.SetTunings(consKp, consKi, consKd);
    }
    else
    {
       //we're far from setpoint, use aggressive tuning parameters
       myPID.SetTunings(Kp, Ki, Kd);
    }
    
    myPID.Compute();
    g=sin(theta*3.14*0.00556)*mg_kt;
    
    ea_nintey=sineArraySize/4.0;
    if (Output < 0)
      lead_lag=-ea_nintey;
    else if (u >= 0)
      lead_lag= ea_nintey;
    
    // PWM
    
    u =  abs(Output); // 10 - (10+x)%


    if (Output+g > 0){
      digitalWrite(4, HIGH);
      digitalWrite(3, LOW);
    } else {
      digitalWrite(4, LOW);
      digitalWrite(3, HIGH);
    }
    currentStepA = a_theta + ea_nintey;
    currentStepB = currentStepA + phaseShift;
    currentStepC = currentStepB + phaseShift;

    if(currentStepA > sineArraySize)  currentStepA = currentStepA - sineArraySize;
    if(currentStepA < 0)  currentStepA = sineArraySize + sineArraySize;
 
    if(currentStepB > sineArraySize)  currentStepB =currentStepB - sineArraySize;
    if(currentStepB < 0)  currentStepB = currentStepB + sineArraySize;
 
    if(currentStepC > sineArraySize)  currentStepC = currentStepC - sineArraySize;
    if(currentStepC < 0) currentStepC =  currentStepC + sineArraySize;

    
    analogWrite(IN1, pwmSin[currentStepA]*u);
    analogWrite(IN2, pwmSin[currentStepB]*u);
    analogWrite(IN3, pwmSin[currentStepC]*u);  
    
//    if (Serial.available() > 0) {
//            // read the incoming byte:
//            float buf = Serial.parseFloat();
//            if(buf != 0){
//              Setpoint = buf;  
//            }
//            
//    }

    int reading = analogRead(A5);
    Setpoint = map(reading, 171, 748, 20, 90);
//
   Serial.print("R: ");
   Serial.print(Setpoint);
   Serial.print(" | O: ");
   Serial.print(u);
   Serial.print(" | T: ");
   Serial.print(" | e_Theta: ");
   Serial.print(e_theta);
   Serial.println(sumTheta);
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
