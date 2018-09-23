#include "PIDController.h"
#include <random>
#include <ctime>
#include <cstdio>

PIDController::PIDController(char * s1, char * s2, char * a1, char * a2)
	: ctrlR(3, 0.5, 0.25), ctrlT(0.75, 0.05, 0.15)
{
	sensor1 = s1;
	sensor2 = s2;
	actuator1 = a1;
	actuator2 = a2;
}

void PIDController::setpoint(double radius, double angle)
{
	setR = radius;
	setT = angle;
}

void PIDController::setRadiusGains(double kp, double ki, double kd)
{
	ctrlR.setKp(kp);
	ctrlR.setKi(ki);
	ctrlR.setKd(kd);
}

void PIDController::setAngleGains(double kp, double ki, double kd)
{
	ctrlT.setKp(kp);
	ctrlT.setKi(ki);
	ctrlT.setKd(kd);
}

void PIDController::step(double dt) {
	double angle1 = (*sensorRef)[sensor1][0] + 3.14 / 2;
	double angle2 = 3.14 / 2 - (*sensorRef)[sensor2][0];

	double totalT = (angle1 - angle2);
	double angleBetween = 3.14 - angle1 - angle2;

	double len1 = 0.1, len2 = 0.2;
	double auxLen = sin(angleBetween / 2) * len1; //length from "knee" to axis of the leg

	double expectedR = len1 * cos(angleBetween / 2) + sqrt(len2 * len2 - auxLen * auxLen);

	double dr = ctrlR.calculateOutput(dt, setR, expectedR);
	double dth = ctrlT.calculateOutput(dt, setT, totalT);

	(*actuatorRef)[actuator1] = dth + dr;
	(*actuatorRef)[actuator2] = dth - dr;
}
