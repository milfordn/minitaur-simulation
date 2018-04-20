#include "PIDController.h"
#include <random>
#include <ctime>
#include <cstdio>

PIDController::PIDController()
	: ctrlR(3, 0, 0.2), ctrlT(0.5, 0, 0.05)
{
	this->setR = 0.15;
	this->setT = 0;
	this->counter = 0;
	srand(time(NULL));
}

void PIDController::step(double dt) {
	double angle1 = (*sensorRef)["thigh1_spos"] + 3.14 / 2;
	double angle2 = (*sensorRef)["thigh2_spos"] - 3.14 / 2;

	//printf("%f, %f", angle1, angle2);

	double totalT = (angle1 + angle2);
	double angleCorrected = (angle1 - totalT);

	double expectedR = 0.1 * sin(angleCorrected) + 0.2 * sin(acos(cos(angleCorrected) / 2));

	/*if (counter > 2) {
		setR = (rand() % 1000) / 1000.0 * 0.1 + 0.1;
		setT = (rand() % 1000) / 1000.0 * 3.14159;
		counter = 0;
	}*/
	setR = 0.075 * cos(counter * 3.14 * 1.75) + 0.2;
	setT = 3.14 / 3 * -sin(counter * 3.14 * 1.75);// +3.14 / 6;

	double dr = ctrlR.calculateOutput(dt, setR, expectedR);
	double dth = ctrlT.calculateOutput(dt, setT, totalT);

	counter += dt;

	printf("%f <-> %f | %f\n", expectedR, setR, totalT);

	(*actuatorRef)["thigh1_a"] = dth + dr;
	(*actuatorRef)["thigh2_a"] = dth - dr;
}