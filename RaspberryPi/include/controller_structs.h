typedef struct ZmqSubFingerMem{
  bool runFlag;
  bool newMessage;
  short fingerSelect;
  short controllerSelect;
  float data1;
  float data2;
  float data3;
  float data4;
  float data5;
  float data6;
  float data7;
  float data8;
  float data9;
  float data10;
  short trajSize;
  float trajTimeStamp[1024];
  float trajPosition[1024];
  float trajVelocity[1024];
  float trajAcceleration[1024];
}ZmqFingerMem;

typedef struct PeripheralFingerMem{
  bool runFlag;
  float jointAngle1;
  float jointAngle2;
  float angularVel1;
  float angularVel2;
  float commandedTorque1;
  float commandedTorque2;
}PeripheralFingerMem;

typedef struct ZmqHandMem{
 ZmqFingerMem finger[7];
}ZmqHandMem;
/*
typedef struct FingerStates{
float angle;
float angVelocity;
float angAcceleration;
float commandedTorque;
float torque;
flaot Empty1;
float empty2;
float empty3;
}FingerStates;
*/
