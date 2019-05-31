#include "zmq.hpp"
#include "flatbuffers/flatbuffers.h"
#include "finger_broadcast_generated.h"
#include <iostream>
#include <stdio.h>
#include "matplotlibcpp.h"
#include <ctime>
#include <unistd.h>
#include <sys/time.h>

#include <cstdlib>
#include <fstream>

namespace plt = matplotlibcpp;


using namespace quad_double_me; // Specified in the schema.

typedef struct InputData{
  short id;
  float angle1;
  float angle2;
  float angVelocity1;
  float angVelocity2;
  float angAcceleration1;
  float angAcceleration2;
  float commandedTorque1;
  float commandedTorque2;
  float torque1;
  float torque2;
  float empty3;
  float empty4;
  float timeStamp;
}InputData;

int main(int argc, char *argv[]){
  timeval a;
  timeval b;
  zmq::context_t context(1);
  zmq::socket_t subscriber(context, ZMQ_SUB);
  zmq::message_t buffer;
  subscriber.setsockopt(ZMQ_SUBSCRIBE,"",0);
  subscriber.connect("tcp://169.254.23.164:5564");
  int n=0;
  int iterations = 10000;
  char* name;
  if (argc >= 3)
  {
    iterations = atoi(argv[1]);
    name = argv[2];
  }
  std::cout << name << std::endl;

  InputData inputData[7];
  std::vector<InputData> fingerStateVector;
  fingerStateVector.reserve(7);
  std::vector<std::vector<InputData>> handStateVector;
  fingerStateVector.reserve(iterations);

  std::vector<float> angle1;
  std::vector<float> angle2;
  std::vector<float> angVelocity1;
  std::vector<float> angVelocity2;
  std::vector<float> angAcceleration1;
  std::vector<float> angAcceleration2;
  std::vector<float> commandedTorque1;
  std::vector<float> commandedTorque2;
  std::vector<float> torque1;
  std::vector<float> torque2;
  std::vector<float> time;;
  angle1.reserve(iterations);
  angle2.reserve(iterations);
  angVelocity1.reserve(iterations);
  angVelocity2.reserve(iterations);
  angAcceleration1.reserve(iterations);
  angAcceleration2.reserve(iterations);
  commandedTorque1.reserve(iterations);
  commandedTorque2.reserve(iterations);
  torque1.reserve(iterations);
  torque2.reserve(iterations);
  time.reserve(iterations);

  gettimeofday(&b, 0);
  bool first = 1;
  while(n<iterations){
      subscriber.recv(&buffer);
      if (first){
        first = 0;
        gettimeofday(&b, 0);
      }
      auto handBroadcastObj = GetHandBroadcast(buffer.data());
      auto handObject = handBroadcastObj->hand();
      gettimeofday(&a, 0);

      for (int i=0; i < handObject->Length(); i++)
      {
        auto fingerStates = handObject->Get(i);
        inputData[i].id =  fingerStates->id();
        inputData[i].angle1 = fingerStates->angle1();
        inputData[i].angle2 = fingerStates->angle2();
        inputData[i].angVelocity1 = fingerStates->angVelocity1();
        inputData[i].angVelocity2 = fingerStates->angVelocity2();
        inputData[i].angAcceleration1 = fingerStates->angAcceleration1();
        inputData[i].angAcceleration2 = fingerStates->angAcceleration2();
        inputData[i].commandedTorque1 = fingerStates->commandedTorque1();
        inputData[i].commandedTorque2 = fingerStates->commandedTorque2();
        inputData[i].torque1 = fingerStates->Empty1();
        inputData[i].torque2 = fingerStates->empty2();
        inputData[i].empty3 = fingerStates->empty3();
        inputData[i].empty4 = fingerStates->empty4();
        inputData[i].timeStamp = a.tv_sec*1.0 + a.tv_usec * 0.000001 - (b.tv_sec*1.0 + b.tv_usec * 0.000001);
        fingerStateVector.push_back(inputData[i]);
        //std::cout << inputData[i].timeStamp << std::endl;
      }
      handStateVector.push_back(fingerStateVector);
      fingerStateVector.clear();
      n=n+1;

      //fingerMem.data1 = messageObj->data1();
  }

  std::ofstream csv_file;
  csv_file.open (name);

  for (int n=0 ; n< iterations; n=n+5){
        angle1.push_back(handStateVector[n][0].angle1);
        csv_file << handStateVector[n][0].angle1 << ",";
        angle2.push_back(handStateVector[n][0].angle2);
        csv_file << handStateVector[n][0].angle2 << ",";
        angVelocity1.push_back(handStateVector[n][0].angVelocity1);
        csv_file << handStateVector[n][0].angVelocity1 << ",";
        angVelocity2.push_back(handStateVector[n][0].angVelocity2);
        csv_file << handStateVector[n][0].angVelocity2 << ",";
        //angAcceleration1.push_back(handStateVector[n][0].angAcceleration1);
        //angAcceleration2.push_back(handStateVector[n][0].angAcceleration2);
        commandedTorque1.push_back(handStateVector[n][0].commandedTorque1);
        csv_file << handStateVector[n][0].commandedTorque1 << ",";
        commandedTorque2.push_back(handStateVector[n][0].commandedTorque2);
        csv_file << handStateVector[n][0].commandedTorque2 << ",";
        //torque1.push_back(handStateVector[n][0].torque1);
        //torque2.push_back(handStateVector[n][0].torque2);
        time.push_back(handStateVector[n][0].timeStamp);
        csv_file << handStateVector[n][0].timeStamp << ",\n";
  }
  csv_file.close();

  std::cout <<"got here"<< std::endl;
  plt::named_plot("Angle1",time, angle1);
  plt::named_plot("Angle2",time, angle2);
  //plt::named_plot("Angular velocity 1",time, angVelocity1);
  //plt::named_plot("Angular velocity 2",time, angVelocity2);
  //plt::named_plot("Angular acceleration 1",time, angAcceleration1);
  //plt::named_plot("Angular acceleration 2",time, angAcceleration2);
  //plt::named_plot("Commanded torque 1",time, commandedTorque1);
  //plt::named_plot("Commanded torque 2",time, commandedTorque2);
  //plt::named_plot("Torque 1",torque1);
  //plt::named_plot("Torque 2",torque2);
  plt::legend();
  plt::show();
  return 0;
}

/*
// Example IDL file for our monster's schema.
namespace quad_double_me;

table HandBroadcast {
  hand:[FingerStates];
}

table FingerStates {
  id:short;
  angle1:float;
  angle2:float;
  angVelocity1:float;
  angVelocity2:float;
  angAcceleration1:float;
  angAcceleration2:float;
  commandedTorque1:float;
  commandedTorque2:float;
  Torque:float;
  Torque:float;
  empty3:float;
  empty4:float;
}

root_type HandBroadcast;
file_identifier "HBRC";
*/
