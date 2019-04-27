#include <SPI.h>

#define V_MOSI  23
#define V_MISO  19
#define V_CLK   18
#define V_CS    5

#define M1_PWM1 0
#define M1_PWM2 2
#define M1_PWM3 4
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

  Serial.begin(115200);
  
}

void loop() {
    int lead_lag;
    double e_theta_0;
    int currentStepA;
    int currentStepB;
    int currentStepC;
    int sineArraySize;
    int increment = 0;
    double e_theta; //electrical angle
    double a_theta; //array angle
    boolean direct = 1; // direction true=forward, false=backward
    int read_pot;
    int start = 1;
    double u = 1;
    //90 degrees angle;
    int ea_nintey = sineArraySize/4;
  double theta, deltaTheta, sumTheta = 0, thetaPrev = 0;
  double Output = 1;

  sineArraySize = sizeof(pwmSin)/sizeof(int); // Find lookup table size
  int phaseShift = sineArraySize / 3.0;         // Find phase shift and initial A, B C phase values
  currentStepA = 0;
  currentStepB = currentStepA + phaseShift;
  currentStepC = currentStepB + phaseShift;
 
  sineArraySize--; // Convert from array Size to last PWM array number

    //pwm
  uint8_t pwmA, pwmB, pwmC;

    // PWM
  sigmaDeltaSetup(CH1, PWM_FRQ);
  sigmaDeltaSetup(CH2, PWM_FRQ);
  sigmaDeltaSetup(CH3, PWM_FRQ);

  sigmaDeltaAttachPin(M1_PWM1, CH1);
  sigmaDeltaAttachPin(M1_PWM2, CH2);
  sigmaDeltaAttachPin(M1_PWM3, CH3);

  pwmA = (pwmSin[currentStepA]);
  pwmB = (pwmSin[currentStepB]);
  pwmC = (pwmSin[currentStepC]);

    sigmaDeltaWrite(M1_PWM1, pwmA);
    sigmaDeltaWrite(M1_PWM2, pwmB);
    sigmaDeltaWrite(M1_PWM3, pwmC);  


  while(1){
        phaseShift = sineArraySize / 3.0;
  currentStepA = 0;
  currentStepB = currentStepA + phaseShift;
  currentStepC = currentStepB + phaseShift;


    vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(V_CS, LOW);
  theta = vspi->transfer(0x00);
  digitalWrite(V_CS, HIGH);
  vspi->endTransaction();

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

    e_theta=4.0*theta-e_theta_0;
    if (e_theta < 0) e_theta=360+e_theta;
    if (e_theta > 360 && e_theta <= 720) e_theta= e_theta - 360;
    else if (e_theta > 720 && e_theta <= 1080) e_theta= e_theta - 720;
    else if (e_theta > 1080)e_theta= e_theta - 1080;

    if (start==1){
      e_theta_0=e_theta;
      start=0;
      e_theta=0;
    }

    a_theta=floor((sineArraySize/360.0)*e_theta);

    ea_nintey=sineArraySize/4.0;
    if (Output < 0)
      lead_lag=-ea_nintey;
    else if (Output >= 0)
      lead_lag= ea_nintey;

    currentStepA = a_theta + lead_lag;
    currentStepB = currentStepA + phaseShift;
    currentStepC = currentStepB + phaseShift;

    if(currentStepA > sineArraySize)  currentStepA = currentStepA - sineArraySize;
    if(currentStepA < 0)  currentStepA = currentStepA + sineArraySize;
 
    if(currentStepB > sineArraySize)  currentStepB =currentStepB - sineArraySize;
    if(currentStepB < 0)  currentStepB = currentStepB + sineArraySize;
 
    if(currentStepC > sineArraySize)  currentStepC = currentStepC - sineArraySize;
    if(currentStepC < 0) currentStepC =  currentStepC + sineArraySize;

    sigmaDeltaWrite(M1_PWM1, pwmSin[currentStepA]);
    sigmaDeltaWrite(M1_PWM2, pwmSin[currentStepB]);
    sigmaDeltaWrite(M1_PWM3, pwmSin[currentStepC]);  

  }
  

}