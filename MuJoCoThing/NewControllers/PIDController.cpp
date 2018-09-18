#include "PIDController.h"
#include <random>
#include <ctime>
#include <cstdio>

PIDController::PIDController()
	: ctrlR(3, 0.5, 0.25), ctrlT(0.75, 0.05, 0.15)
{
	this->setR = 0.15;
	this->setT = 0;
	this->counter = 0;
}

void PIDController::step(double dt) {
	double angle1 = (*sensorRef)["thigh1_spos"][0] + 3.14 / 2;
	double angle2 = 3.14 / 2 - (*sensorRef)["thigh2_spos"][0];

	double totalT = (angle1 - angle2);
	double angleBetween = 3.14 - angle1 - angle2;

	double len1 = 0.1, len2 = 0.2;
	double auxLen = sin(angleBetween / 2) * len1; //length from "knee" to axis of the leg

	double expectedR = len1 * cos(angleBetween / 2) + sqrt(len2 * len2 - auxLen * auxLen);

	//setR = 0.075 * cos(counter * 3.14 * 1.75) + 0.2;
	//setT = 3.14 / 3 * -sin(counter * 3.14 * 1.75);// +3.14 / 6;
	if (counter < 0.5) {
		setR = 0.125;
		setT = 3.14 / 4;
	}
	else if (counter < 1) {
		setR = 0.225;
		setT = 3.14 / 4;
	}
	else if (counter < 1.5) {
		setR = 0.225;
		setT = -3.14 / 4;
	}
	else if (counter < 2) {
		setR = 0.125;
		setT = -3.14 / 4;
	}
	else counter = 0;

	double dr = ctrlR.calculateOutput(dt, setR, expectedR);
	double dth = ctrlT.calculateOutput(dt, setT, totalT);

	counter += dt;

	(*actuatorRef)["thigh1_a"] = dth + dr;
	(*actuatorRef)["thigh2_a"] = dth - dr;
}
