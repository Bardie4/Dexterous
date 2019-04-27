#include <MagAlpha.h>
#include <PID_v1.h> // OPEN LOOP

//Check https://www.arduino.cc/en/reference/SPI for SPI signals connections

#define UART_BAUDRATE       115200        //UART data rate in bits per second (baud)
#define SPI_SCLK_FREQUENCY  10000000      //SPI SCLK Clock frequency in Hz
#define SPI_CS_PIN          7             //SPI CS pin

MagAlpha magAlpha;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, 1, 0, 0, DIRECT);

void setup() {
  // put your setup code here, to run once:
  //Set the SPI SCLK frequency, SPI Mode and CS pin
  magAlpha.begin(SPI_SCLK_FREQUENCY, MA_SPI_MODE_3, SPI_CS_PIN);
  //Set the Serial Communication used to report the angle
  Serial.begin(UART_BAUDRATE);

  // PID
  Input = analogRead(0);
  Setpoint = 180;
  myPID.SetOutputLimits(-80,80);
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  int PINccw = 3, PINcw = 4;
  int PINspeed = 5;
  int motorSpeed = 50;
  
  while(1){
    digitalWrite(PINccw, LOW);
    digitalWrite(PINcw, HIGH);

    analogWrite(PINspeed, motorSpeed);


    if (Serial.available() > 0) {
        // read the incoming byte:
        float buf = Serial.parseInt();
        if(buf != 0){
          motorSpeed = buf;  
          
        }
    }
    Serial.println(motorSpeed);
  }
}
