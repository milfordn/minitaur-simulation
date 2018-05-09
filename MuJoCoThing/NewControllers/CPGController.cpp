#include "./CPGController.h"
#include <iostream>
#include <math.h>
using std::cout;
using std::endl;
//7 params per CPG
//double params[28] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
CPGController::CPGController(double params[28]){
  int param_num = 7;
  for(int i = 0; i < 4; i++){
    cpg[i] = new CPGNode(params[i*param_num + 0], params[i*param_num + 1], params[i*param_num + 2], params[i*param_num + 3], params[i*param_num + 4], params[i*param_num + 5], params[i*param_num + 6]);
  }
  for(int i = 0; i < 8; i++){
    motors[i] = new pid(100, 0, 0.1);
  }

}
void CPGController::step(double dt){
  double pitch = (*sensorRef)["body_gyro"][0];
  double roll = (*sensorRef)["body_gyro"][1];
  double yaw = (*sensorRef)["body_gyro"][2];

  //set motor positions
  double L1 = 0.1;
  double L2 = 0.2;
  for(int i = 0; i < 8; i+=2){
    cpg[i/2]->step(dt);
    double desiredAngle = 3.14159/2 + cpg[i/2]->getAngle();
    double desiredLength = 0.191 + 0.1 * cpg[i/2]->getLength();
    double currentm1pos = (*sensorRef)[sensorNames[i]][0];
    double currentm2pos = (*sensorRef)[sensorNames[i+1]][0];
    double currentAngle = (currentm1pos + currentm2pos)/2; //The current angle of the leg
    double currentLength = L1 * sin(currentAngle - currentm1pos) + sqrt(L2*L2 - L1*L1*cos(currentAngle - currentm1pos)*cos(currentAngle - currentm1pos)); //The current distance from the motors to the end effector
    double m1pos = desiredAngle - asin((desiredLength*desiredLength + L1*L1 - L2*L2)/(2 * L1 * desiredLength)); //desired pos of motor1
    double m2pos = desiredAngle + asin((desiredLength*desiredLength + L1*L1 - L2*L2)/(2 * L1 * desiredLength)); //desired pos of motor2

    double output_1 = motors[i]->calculateOutput(time, m1pos, currentm1pos);
    double output_2 = motors[i+1]->calculateOutput(time, m2pos, currentm2pos);

    (*actuatorRef)[motorNames[i]] = output_1;
    (*actuatorRef)[motorNames[i+1]] = output_2;
    //if(i == 0) cout << sensorNames[i] << ": " << currentm1pos << ", " << sensorNames[i+1] << ": " << currentm2pos << endl;

  }
  time += dt;
}
