#include "LegControllerCPG.h"
#include <cstdio>

LegControllerCPG::LegControllerCPG(char * s1, char * s2, char * m1, char * m2, CPGNode * controller) :
	innerCtrl(s1, s2, m1, m2) {
	radiusController = controller;
	sensor1 = s1;
	sensor2 = s2;
	motor1 = m1;
	motor2 = m2;

	innerCtrl.setRadiusGains(15, 1, 0.2);
	innerCtrl.setAngleGains(2, 0.05, 0.1);
}

void LegControllerCPG::setSensorRef(unordered_map<string, vector<double>>* ref)
{
	Controller::setSensorRef(ref);
	innerCtrl.setSensorRef(ref);
}

void LegControllerCPG::setActuatorRef(unordered_map<string, double>* ref)
{
	Controller::setActuatorRef(ref);
	innerCtrl.setActuatorRef(ref);
}

void LegControllerCPG::step(double dt) {
	radiusController->step(dt);

	double y = radiusController->getLength();
	double x = -radiusController->getAngle();

	double setT = x;
	double setR = y > 0 ? 0.125 : 0.25;

	innerCtrl.setpoint(setR, setT);
	innerCtrl.step(dt);
}