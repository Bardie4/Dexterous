#include <MagAlpha.h>
#include <PID_v1.h>

//Check https://www.arduino.cc/en/reference/SPI for SPI signals connections

#define UART_BAUDRATE       115200        //UART data rate in bits per second (baud)
#define SPI_SCLK_FREQUENCY  10000000      //SPI SCLK Clock frequency in Hz
#define SPI_CS_PIN          10             //SPI CS pin

MagAlpha magAlpha;

#define PWM 3
#define DIR 4
#define ENA 5

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp = 0.1, Ki = 0.0, Kd = 0.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  // put your setup code here, to run once:
  //Set the SPI SCLK frequency, SPI Mode and CS pin
  magAlpha.begin(SPI_SCLK_FREQUENCY, MA_SPI_MODE_3, SPI_CS_PIN);
  //Set the Serial Communication used to report the angle
  Serial.begin(UART_BAUDRATE);

  // PID
  Input = analogRead(0);
  Setpoint = 70;
  myPID.SetOutputLimits(-40,40);
  myPID.SetMode(AUTOMATIC);

  // Motor enable
  digitalWrite(ENA, HIGH);
  
}

void loop() {
  double theta, deltaTheta, sumTheta = 0, thetaPrev = 0;
  int reading;
  double u;
  double g;
  double error, prev_error;
  int pulse = 0;

  while(1){

    // Read angle
    theta = magAlpha.readAngle();

    // Angle change
    deltaTheta = theta - thetaPrev;

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
    
    reading = analogRead(A5);
    Setpoint = mapfloat(reading, 177, 748, 20, 280);
    //myPID.SetTunings(Kp, Ki, Kd);
    myPID.Compute();

    u = abs(Output);
    
    // Direction of motor
    if (Output > 0){
      digitalWrite(DIR, HIGH);
    } else {
      digitalWrite(DIR, LOW);
    }

    // Send PWM to motor
    error = abs(Setpoint-Input);
    if (error <= prev_error + 1) {
      analogWrite(PWM, u + 25);
    } else if (pulse > 100) {
      analogWrite(PWM, 60);
      pulse = 0;
    }
    pulse++;
    prev_error = error;
    
     // 5 & 6 have 1kHz PWM

    
//    if (Serial.available() > 0) {
//            // read the incoming byte:
//            float buf = Serial.parseFloat();
//            if(buf != 0){
//              Setpoint = buf;  
//            }
//            
//    }
  Serial.println(theta);
  
// Serial.println(micros());
//    Serial.print(" | Ana: ");
//    Serial.print(reading);
//    Serial.print(" | Kp : ");
//    Serial.println(Kp);



  // 47 deg - 130 deg
  }
}

float mapfloat(long x, long in_min, long in_max, float out_min, float out_max)
{
 return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
