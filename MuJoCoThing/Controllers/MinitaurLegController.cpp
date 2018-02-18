#include "MinitaurLegController.h"
#include <cstdio>
#include <cmath>

MinitaurLegController::MinitaurLegController(const char * f) 
: ModelController (f), ctrl1(0, 0, 0, 0, 0), ctrl2(0, 0, 0, 0, 0){

	int sensor1 = mj_name2id(this->model, mjtObj::mjOBJ_SENSOR, "thigh1_spos");
	int sensor2 = mj_name2id(this->model, mjtObj::mjOBJ_SENSOR, "thigh2_spos");
	int motor1 = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, "thigh1_a");
	int motor2 = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, "thigh2_a");

	this->ctrl1 = AutoPID(-0.01, -0.00005, -10, motor1, sensor1);
	this->ctrl2 = AutoPID(-0.01, -0.00005, -10, motor2, sensor2);

	this->angle = 0;
}

void MinitaurLegController::step() {
	ctrl1.run(data, angle);
	ctrl2.run(data, -angle);

	double angle1 = data->sensordata[mj_name2id(model, mjtObj::mjOBJ_SENSOR, "thigh1_spos")];
	double angle2 = data->sensordata[mj_name2id(model, mjtObj::mjOBJ_SENSOR, "thigh2_spos")];
	double angleAvg = (angle1 - angle2) / 2;
	double expectedR = 0.1 * sin(angleAvg) + 0.2 * sin(acos(cos(angleAvg) / 2));

	printf("%f | %f\n", data->xpos[mj_name2id(model, mjtObj::mjOBJ_BODY, "foot") * 3 + 2], 0.459 - expectedR);
	//printf("%f | %f -> %f | %f\n", data->sensordata[0], data->sensordata[1], data->ctrl[0], data->ctrl[1]);
}

void MinitaurLegController::keyboardCallback(GLFWwindow*, int key, int, int act, int) {
	if (key == GLFW_KEY_W && act == GLFW_PRESS)
		angle += mjPI / 6;
	if (key == GLFW_KEY_S && act == GLFW_PRESS)
		angle -= mjPI / 6;
}