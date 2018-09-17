#include "LegControllerCPG.h"
#include <cstdio>

LegControllerCPG::LegControllerCPG(char * s1, char * s2, char * m1, char * m2, CPGNode * controller) :
	ctrlR(10, 0, 0.2), ctrlT(0.75, 0, 0.05) {
	radiusController = controller;
	sensor1 = s1;
	sensor2 = s2;
	motor1 = m1;
	motor2 = m2;
}

void LegControllerCPG::step(double dt) {
	double angle1 = (*sensorRef)[sensor1][0] + 3.14 / 2;
	double angle2 = (*sensorRef)[sensor2][0] - 3.14 / 2;

	double measuredT = (angle1 + angle2);
	double angleCorrected = (angle1 - measuredT);

	double measuredR = 0.1 * sin(angleCorrected) + 0.2 * sin(acos(cos(angleCorrected) / 2));

	radiusController->step(dt);

	double y = radiusController->getLength();
	double x = -radiusController->getAngle();

	double setT = x;
	double setR = y > 0 ? 0.125 : 0.25;

	double pwrRadius = ctrlR.calculateOutput(dt, setR, measuredR);
	double pwrAngle = ctrlT.calculateOutput(dt, setT, measuredT);

	(*actuatorRef)[motor1] = pwrAngle + pwrRadius;
	(*actuatorRef)[motor2] = pwrAngle - pwrRadius;
}