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
    cpg[i] = new CPGNode(params[0], params[1], params[2], params[3], params[4], params[i*param_num + 5], params[i*param_num + 6]);
	legs[i] = new LegControllerCPG(sensorNames[i * 2], sensorNames[i * 2 + 1], motorNames[i * 2], motorNames[i * 2 + 1], cpg[i]);
	legs[i]->setSensorRef(sensorRef);
	legs[i]->setActuatorRef(actuatorRef);
  }

  pitch = 0;
  tick = 0;
}
void CPGController::step(double dt){
  double dps = 35;
  double gyroX = (*sensorRef)["body_gyro"][0]/dps;
  double gyroY = (*sensorRef)["body_gyro"][1]/dps;
  double gyroZ = (*sensorRef)["body_gyro"][2]/dps;
  double accelX = (*sensorRef)["body_accel"][0];
  double accelY = (*sensorRef)["body_accel"][1];
  double accelZ = (*sensorRef)["body_accel"][2];
  xFinal = (*sensorRef)["body_pos"][0];
  yFinal = (*sensorRef)["body_pos"][1];
  zFinal = (*sensorRef)["body_pos"][2];

  if(zFinal < 0.075) touchedGround = true;
  if(zFinal > 0.3) leftGround = true;

  pitch += gyroY;
  roll += gyroX;
  yaw += gyroZ;

  //gimbal_lock_y += roll * sin(gyroZ);
  //gimbal_lock_x += pitch * sin(gyroZ);

  //double pitchEstimate = pitch + gimbal_lock_y;
  //double rollEstimate = roll + gimbal_lock_x;

  double aTotal = sqrt((accelX*accelX)+(accelY*accelY)+(accelZ*accelZ));
//  cout << aTotal << endl;
  if(tick == 0) aTotal = -9.806;

  double accBiasRoll = asin(accelX/aTotal);
  double accBiasPitch = asin(accelY/aTotal);
  pitch = pitch * 0.9992 + accBiasPitch * 0.0008;
  roll = roll * 0.9992 + accBiasRoll * 0.0008;

  //set motor positions
  double L1 = 0.1;
  double L2 = 0.2;
  //for(int i = 0; i < 8; i+=2){
  //  cpg[i/2]->step(dt);
  //  double desiredAngle = 3.14159/2 + cpg[i/2]->getAngle();
  //  double desiredLength;
  //  if(cpg[i/2]->getLength() > 1) desiredLength = L2 + L1;
  //  else if(cpg[i/2]->getLength() < -1) desiredLength = L2 - L1;
  //  else desiredLength = L2 + L1 * cpg[i/2]->getLength();
  //  double currentm1pos = (*sensorRef)[sensorNames[i]][0];
  //  double currentm2pos = (*sensorRef)[sensorNames[i+1]][0];
  //  double currentAngle = (currentm1pos + currentm2pos)/2;
  //  double currentLength = L1 * sin(currentAngle - currentm1pos) + sqrt(L2*L2 - L1*L1*cos(currentAngle - currentm1pos)*cos(currentAngle - currentm1pos)); //The current distance from the motors to the end effector
  //  double m1pos = desiredAngle - asin((desiredLength*desiredLength + L1*L1 - L2*L2)/(2 * L1 * desiredLength)); //desired pos of motor1
  //  double m2pos = desiredAngle + asin((desiredLength*desiredLength + L1*L1 - L2*L2)/(2 * L1 * desiredLength)); //desired pos of motor2
  //}

  for (int i = 0; i < 4; i++) {
	  legs[i]->step(dt);
  }

  pitchVariance += pitch*pitch;
  rollVariance += roll*roll;
  yawVariance += yaw*yaw;
  tick++;
  time += dt;
}

void CPGController::setSensorRef(unordered_map<string, vector<double>>* ref)
{
	Controller::setSensorRef(ref);

	for (int i = 0; i < 4; i++)
		legs[i]->setSensorRef(ref);
}

void CPGController::setActuatorRef(unordered_map<string, double>* ref)
{
	Controller::setActuatorRef(ref);

	for (int i = 0; i < 4; i++)
		legs[i]->setActuatorRef(ref);
}

double CPGController::exit(){
  pitchVariance /= tick;
  rollVariance /= tick;
  yawVariance /= tick;
  double distanceTraveled = xFinal*xFinal - yFinal*yFinal;
  double reward = distanceTraveled;//(10.0/(pitchVariance) + 10.0/(rollVariance) + 2.0/(yawVariance)) * distanceTraveled/time;
  if(touchedGround || leftGround) reward = 0;
  return reward;
}
