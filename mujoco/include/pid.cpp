#include "pid.h"
#include "stdio.h"
#include "stdlib.h"
#include <iostream>
#include "math.h"
pid::pid(double kp, double ki, double kd){
    KP = kp;
    KI = ki;
    KD = kd;
    lastError = 0;
    integral = 0;
    lastPosition = 0;
}
double pid::calculateOutput(double position, double setpoint){
    double error = position - setpoint;
    integral += error;
    double derivative = position - lastPosition;
    
    lastPosition = position;
    std::cout << "ERROR: " << error << " INTEGRAL: " << integral << " DERIVATIVE: " << derivative << std::endl;
    return error*KP + integral*KI + derivative*KD;
}