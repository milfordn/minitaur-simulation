#include "LegControllerCPG.h"
#include <cstdio>

LegControllerCPG::LegControllerCPG(char * s1, char * s2, char * m1, char * m2, CPGNode * controller) :
	anglectrl1(1, 0, 0.05), anglectrl2(1, 0, 0.05) {
	radiusController = controller;
	sensor1 = s1;
	sensor2 = s2;
	motor1 = m1;
	motor2 = m2;
}

void LegControllerCPG::step(double dt) {
	double angle1 = (*sensorRef)[sensor1][0];
	double angle2 = (*sensorRef)[sensor2][0];

	radiusController->step(dt);
	double nextAngle1 = radiusController->getAngle() + 3.14 / 2;

	(*actuatorRef)[motor1] = anglectrl1.calculateOutput(dt, -nextAngle1, angle1);
	(*actuatorRef)[motor2] = anglectrl2.calculateOutput(dt, nextAngle1, angle2);

	printf("%f -> %f | %f -> %f\n", angle1, nextAngle1, angle2, -nextAngle1);
}