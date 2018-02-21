#include "pid.h"
#include "math.h"
#include <chrono>
#include <iostream>

pid::pid(double kp, double ki, double kd){
    KP = kp;
    KI = ki;
    KD = kd;
    lastError = 0;
    integral = 0;
    lastPosition = 0;
	lastTick = 0;
}
double pid::calculateOutput(unsigned long tick, double setpoint, double position){
	int deltaT = tick - lastTick;
	
    double error = setpoint - position;
    integral += error * deltaT;
    double derivative = (lastPosition - position)/deltaT;
    
    lastPosition = position;
	lastTick = tick;
    return error*KP + integral*KI + derivative*KD;
}
double pid::calculateOutput(unsigned long tick, double setpoint, double position, double velocity){
	int deltaT = tick - lastTick;
	
	double error = setpoint - position;
    integral += error * deltaT;
    double derivative = velocity/deltaT;
    
	lastTick = tick;
    return error*KP + integral*KI + derivative*KD;
}