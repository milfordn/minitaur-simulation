#include "./CPGController.h"
#include <iostream>
#include <math.h>
using std::cout;
using std::endl;
//7 params per CPG
//double params[28] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
CPGController::CPGController(double params[28]){
  backLeft = new CPGNode(params[0], params[1], params[2], params[3], params[4], params[5], params[6]);
  backLeft = new CPGNode(params[7], params[8], params[9], params[10], params[11], params[12], params[13]);
  backLeft = new CPGNode(params[14], params[15], params[16], params[17], params[18], params[19], params[20]);
  backLeft = new CPGNode(params[21], params[22], params[23], params[24], params[25], params[26], params[27]);
  for(int i = 0; i < 8; i++){
    motors[i] = new pid(0.01, 0, 0.0);
  }

}
void CPGController::step(double dt){
  cout << "here" << endl;
  double pitch = (*sensorRef)["body_gyro"][0];
  double roll = (*sensorRef)["body_gyro"][1];
  double yaw = (*sensorRef)["body_gyro"][2];
  double desiredAngle = 0.2*sin(time/10);
  double desiredLength = 0.191;

  //set motor positions
  double L1 = 0.1;
  double L2 = 0.2;
  for(int i = 0; i < 0; i+=2){
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
