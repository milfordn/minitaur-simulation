#include "pid.h"
#include "math.h"
#include <iostream>
using std::cout;
using std::endl;

pid::pid(double kp, double ki, double kd){
  KP = kp;
  KI = ki;
  KD = kd;
  lastError = 0;
  integral = 0;
  lastPosition = 0;
  lower = 0;
  upper = 0;
  lastTick = 0;
}

void pid::limitOutput(double l, double u){
  lower = l;
  upper = u;
}

double pid::calculateOutput(double tick, double setpoint, double position){
	double deltaT = tick;
	double error = setpoint - position;

	double derivative;
	if(deltaT == 0){
		derivative = 0;
	}else{
		integral += error * deltaT;
		derivative = (lastPosition - position)/deltaT;
	}
	if(isnan(integral)) integral = 0;

	lastTick = tick;
	lastPosition = position;

	double output = error * KP + integral * KI;

	if (upper != 0 && output > upper) {
		integral = (upper - (error * KP)) / KI; //back-calculate what the integral should be at this point
		if (integral < 0) integral = 0;
	}
	if (lower != 0 && output <= lower) {
		integral = (lower - (error * KP)) / KI;
		if (integral < 0) integral = 0;
	}

	return output + derivative * KD; //add derivative after limit calculation to allow for damping (assuming KD > 0, which it always should be)
}

double pid::calculateOutput(double tick, double setpoint, double position, double velocity){
	double deltaT = tick;
	double error = setpoint - position;
    integral += error * deltaT;

    double derivative;
	if(deltaT == 0){
		derivative = 0;
	}else{
    	derivative = velocity/deltaT;
	}

	lastTick = tick;
    return error*KP + integral*KI + derivative*KD;
}

void pid::setPgain(double p){
	KP = p;
}
