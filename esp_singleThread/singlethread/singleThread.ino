
#include <SPI.h>

#define V_MOSI  23
#define V_MISO  19
#define V_CLK   18
#define V_CS    5

#define nReset  26

#define M1_PWM1 2
#define M1_PWM2 4
#define M1_PWM3 21
#define CH1     0
#define CH2     1
#define CH3     2
#define M1_EN   16
#define PWM_FRQ 250000

SPIClass * vspi = NULL;
static const int spiClk = 20*1000*1000; // 20 MHz

const uint8_t pwmSin[] = {128,131,134,137,140,143,146,149,152,155,158,162,165,167,170,173,176,179,182,185,188,190,193,196,198,201,203,206,208,211,213,215,218,220,222,224,226,228,230,232,234,235,237,238,240,241,243,244,245,246,248,249,250,250,251,252,253,253,254,254,254,255,255,255,255,255,255,255,254,254,254,253,253,252,251,250,250,249,248,246,245,244,243,241,240,238,237,235,234,232,230,228,226,224,222,220,218,215,213,211,208,206,203,201,198,196,193,190,188,185,182,179,176,173,170,167,165,162,158,155,152,149,146,143,140,137,134,131,128,124,121,118,115,112,109,106,103,100,97,93,90,88,85,82,79,76,73,70,67,65,62,59,57,54,52,49,47,44,42,40,37,35,33,31,29,27,25,23,21,20,18,17,15,14,12,11,10,9,7,6,5,5,4,3,2,2,1,1,1,0,0,0,0,0,0,0,1,1,1,2,2,3,4,5,5,6,7,9,10,11,12,14,15,17,18,20,21,23,25,27,29,31,33,35,37,40,42,44,47,49,52,54,57,59,62,65,67,70,73,76,79,82,85,88,90,93,97,100,103,106,109,112,115,118,121,124};

void setup() {
  vspi = new SPIClass(VSPI);

  vspi->begin(V_CLK, V_MISO, V_MOSI, V_CS);

  pinMode(V_CS, OUTPUT); //VSPI SS

  pinMode(nReset, OUTPUT);
  digitalWrite(nReset, HIGH);

  pinMode(M1_EN, OUTPUT);
  digitalWrite(M1_EN, HIGH);


  Serial.begin(115200);
  
}

// the loop function runs over and over again until power down or reset
void loop() {
  uint16_t uiAngle;

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

    // PWM
  // sigmaDeltaSetup(CH1, PWM_FRQ);
  // sigmaDeltaSetup(CH2, PWM_FRQ);
  // sigmaDeltaSetup(CH3, PWM_FRQ);
  ledcSetup(CH1, PWM_FRQ, 8);
  ledcSetup(CH2, PWM_FRQ, 8);
  ledcSetup(CH3, PWM_FRQ, 8);
  
  ledcAttachPin(M1_PWM1, CH1);
  ledcAttachPin(M1_PWM2, CH2);
  ledcAttachPin(M1_PWM3, CH3);

  // sigmaDeltaAttachPin(M1_PWM1, CH1);
  // sigmaDeltaAttachPin(M1_PWM2, CH2);
  // sigmaDeltaAttachPin(M1_PWM3, CH3);

  pwmA = (pwmSin[currentStepA]);
  pwmB = (pwmSin[currentStepB]);
  pwmC = (pwmSin[currentStepC]);
  
  // sigmaDeltaWrite(CH1, 0);
  // sigmaDeltaWrite(CH2, 85);
  // sigmaDeltaWrite(CH3, 170);

  ledcWrite(CH1, 0);
  ledcWrite(CH2, 85);
  ledcWrite(CH3, 170);  

  Serial.println("Synchronizing");
  delay(5000);

  Serial.println("start");

  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(V_CS, LOW);
  theta_0_mek = vspi->transfer(0x00);
  digitalWrite(V_CS, HIGH);
  vspi->endTransaction();

  theta_0 = theta_0_mek << 2;
  startPos = 255 - theta_0 + 1;  
  Serial.println(theta_0);
  

  while(1){
    vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
    digitalWrite(V_CS, LOW);
    theta_raw = vspi->transfer(0x00);
    digitalWrite(V_CS, HIGH);
    vspi->endTransaction();

    if (u <= 0) lead_lag = phaseShift8_90;
    else if (u > 0) lead_lag = phaseShift8_90_minus;
    
    // Read angle
    theta_multiplied = theta_raw << 2;        //Multiplied by 4 for electrical angle.
    
    theta_multiplied += startPos;
    theta_multiplied += lead_lag;             //Adding lead/lag angle based on direction
    currentStepA = theta_multiplied;          //Masked by conversion back to 16 bit to cycle around 360 deg 4 times in one mechanical rotation. (Modulus 360 degrees)
    currentStepB = currentStepA + phaseShift8_120;
    currentStepC = currentStepB + phaseShift8_120;

    outputScale = fabs(u);

    //Output
    pwmA = (int)(pwmSin[currentStepA]);
    pwmB = (int)(pwmSin[currentStepB]);
    pwmC = (int)(pwmSin[currentStepC]);
    // sigmaDeltaWrite(CH1, pwmA);
    // sigmaDeltaWrite(CH2, pwmB);
    // sigmaDeltaWrite(CH3, pwmC);
    ledcWrite(CH1, pwmA);  
    ledcWrite(CH2, pwmB);
    ledcWrite(CH3, pwmC);  

    // Serial.print(" | u : ");
    // Serial.print(u);
    // Serial.print(" | OutputScale: ");
    // Serial.print(outputScale);
    // Serial.print(" | theta mult.: ");
    // Serial.print(theta_multiplied);
    // Serial.print(" | theta raw: ");
    // Serial.print(theta_raw);
    // Serial.print(" | PWM : ");
    Serial.print(pwmA);
    Serial.print(" | ");
    Serial.print(pwmB);
    Serial.print(" | ");
    Serial.println(pwmC);
  }
}