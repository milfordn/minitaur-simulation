#include "MinitaurLegController.h"
#include <cstdio>

MinitaurLegController::MinitaurLegController(const char * f) 
: ModelController (f), ctrl1(0, 0, 0, 0, 0), ctrl2(0, 0, 0, 0, 0){
	pid p1(0.1, 0, 0.01);
	pid p2(0.1, 0, 0.01);

	int sensor1 = mj_name2id(this->model, mjtObj::mjOBJ_SENSOR, "thigh1_spos");
	int sensor2 = mj_name2id(this->model, mjtObj::mjOBJ_SENSOR, "thigh2_spos");
	int motor1 = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, "thigh1_a");
	int motor2 = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, "thigh2_a");

	this->ctrl1 = AutoPID(-0.025, 0, -0.25, motor1, sensor1);
	this->ctrl2 = AutoPID(-0.025, 0, -0.25, motor2, sensor2);

	this->angle = 0;
}

void MinitaurLegController::step() {
	ctrl1.run(data, angle);
	ctrl2.run(data, angle);

	//printf("%f | %f -> %f | %f\n", data->sensordata[0], data->sensordata[1], data->ctrl[0], data->ctrl[1]);
}

void MinitaurLegController::keyboardCallback(GLFWwindow*, int key, int, int act, int) {
	if (key == GLFW_KEY_W && act == GLFW_PRESS)
		angle += 0.1;
	if (key == GLFW_KEY_S && act == GLFW_PRESS)
		angle -= 0.1;
}