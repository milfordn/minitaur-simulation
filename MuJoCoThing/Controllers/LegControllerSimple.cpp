#include "LegControllerSimple.h"
#include <cstdio>
#include <cmath>

LegControllerSimple::LegControllerSimple(const char * f) 
: ModelController (f), ctrlR(0, 0, 0), ctrlT(0, 0, 0){

	sensor1 = mj_name2id(this->model, mjtObj::mjOBJ_SENSOR, "thigh1_spos");
	sensor2 = mj_name2id(this->model, mjtObj::mjOBJ_SENSOR, "thigh2_spos");
	motor1 = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, "thigh1_a");
	motor2 = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, "thigh2_a");

	this->ctrlR = pid(0.5, 0, 50);
	this->ctrlT = pid(-0.1, 0, -10);

	this->setR = 0.2;
	this->setT = 0;
}

void LegControllerSimple::step() {
	double angle1 = data->sensordata[sensor1];
	double angle2 = data->sensordata[sensor2];
	double angleAvg = (angle1 - angle2) / 2;

	double expectedR = 0.1 * sin(angleAvg) + 0.2 * sin(acos(cos(angleAvg) / 2));
	double totalT = (angle1 + angle2);

	setR = 0.05 * cos(data->time * mjPI * 1.75) + 0.2;
	setT = mjPI / 3 * -sin(data->time * mjPI * 1.75) + mjPI / 6;

	double dr = ctrlR.calculateOutput(data->time, expectedR, setR);
	double dt = ctrlT.calculateOutput(data->time, totalT, setT);

	data->ctrl[motor1] = dt - dr;
	data->ctrl[motor2] = dt + dr;

	printf("%f <-> %f | %f\n", expectedR, setR, totalT);
	//printf("%f | %f -> %f | %f\n", data->sensordata[0], data->sensordata[1], data->ctrl[0], data->ctrl[1]);
}

void LegControllerSimple::keyboardCallback(GLFWwindow*, int key, int, int act, int) {
	if (act != GLFW_PRESS) return;
	if (key == GLFW_KEY_W)
		setR -= 0.025;
	if (key == GLFW_KEY_S)
		setR += 0.025;
	if (key == GLFW_KEY_A)
		setT += mjPI / 6;
	if (key == GLFW_KEY_D)
		setT -= mjPI / 6;
}