typedef struct ZmqFingerMem{
  bool runFlag;
  bool newMessage;
  short fingerSelect;
  short controllerSelect;
  float data1;
  float data2;
  float data3;
  float data4;
  short trajSize;
  float trajTimeStamp[1024];
  float trajPosition[1024];
  float trajVelocity[1024];
  float trajAcceleration[1024];
}ZmqFingerMem;

typedef struct SpiFingerMem{
  bool runFlag;
  float jointAngle1;
  float jointAngle2;
  float angularVel1;
  float angularVel2;
  float commandedTorque1;
  float commandedTorque2;
}SpiFingerMem;

typedef struct ZmqHandMem{
 ZmqFingerMem finger[7];
}ZmqHandMem;

typedef struct SpiHandMem{
 SpiFingerMem finger[7];
}SpiHandMem;

/*
typedef struct chipSelect{
  short csJointSensor1;
  short csJointSensor2;
  short csTorqueController;
}chipSelect;

typedef struct csList{
  chipSelect finger[7];
}csList;
*/
