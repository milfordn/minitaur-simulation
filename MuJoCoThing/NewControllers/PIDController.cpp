#include "PIDController.h"
#include <random>
#include <ctime>

PIDController::PIDController()
	: ctrlR(0.5, 0, 5), ctrlT(-0.1, 0, -10)
{
	this->setR = 0.15;
	this->setT = 0;
	this->counter = 0;
	srand(time(NULL));
}

void PIDController::step(double dt) {
	double angle1 = (*sensorRef)["thigh1_spos"];
	double angle2 = (*sensorRef)["thigh2_spos"];
	double angleAvg = (angle1 - angle2);

	double expectedR = 0.1 * sin(angleAvg) + 0.2 * sin(acos(cos(angleAvg) / 2));
	double totalT = (angle1 + angle2);

	if (counter > 2) {
		setR = (rand() % 1000) / 1000.0 * 0.1 + 0.1;
		setT = (rand() % 1000) / 1000.0 * 3.14159;
	}

	double dr = ctrlR.calculateOutput(dt, expectedR, setR);
	double dth = ctrlT.calculateOutput(dt, totalT, setT);

	(*actuatorRef)["thigh1_a"] = dth - dr;
	(*actuatorRef)["thigh2_a"] = dr - dth;
}