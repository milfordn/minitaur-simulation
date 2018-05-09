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
    motors[i] = new pid(100, 0, 0.1);
  }
  char * mnames[8] = {
    (char*)"thigh1FL_a",
    (char*)"thigh2FL_a",
    (char*)"thigh1FR_a",
    (char*)"thigh2FR_a",
    (char*)"thigh1BL_a",
    (char*)"thigh2BL_a",
    (char*)"thigh1BR_a",
    (char*)"thigh2BR_a",
  };
  for(int i = 0; i < 8; i++){
    motorNames[i] = mnames[i];
  }

}
void CPGController::step(double dt){
  double pitch = (*sensorRef)["body_gyro"];
  double desiredAngle = 3.14159;
  double desiredLength = 0.191;
  //set motor positions
  double L1 = 0.1;
  double L2 = 0.2;
  for(int i = 0; i < 8; i+=2){
    double currentm1pos = (*sensorRef)[motorNames[i]];
    double currentm2pos = (*sensorRef)[motorNames[i+1]];
    double currentAngle = (currentm1pos + currentm2pos)/2; //The current angle of the leg
    double currentLength = L1 * sin(currentAngle - currentm1pos) + sqrt(L2*L2 - L1*L1*cos(currentAngle - currentm1pos)*cos(currentAngle - currentm1pos)); //The current distance from the motors to the end effector
    double m1pos = desiredAngle + asin((desiredLength*desiredLength + L1*L1 - L2*L2)/(2 * L1 * desiredLength)); //desired pos of motor1
    double m2pos = desiredAngle - asin((desiredLength*desiredLength + L1*L1 - L2*L2)/(2 * L1 * desiredLength)); //desired pos of motor2
    double output_1 = motors[i]->calculateOutput(time, m1pos, currentm1pos);
    double output_2 = motors[i+1]->calculateOutput(time, m2pos, currentm2pos);
    (*actuatorRef)[motorNames[i]] = output_2;
    (*actuatorRef)[motorNames[i+1]] = output_1;
    cout << output_1 << endl;

  }
  time += dt;
}
