#include "LegControllerCPG.h"
#include <cstdio>

LegControllerCPG::LegControllerCPG(char * f)
	: ModelController(f), radiusController(50, 1, 4) {
	sensor1 = mj_name2id(this->model, mjtObj::mjOBJ_SENSOR, "thigh1_spos");
	sensor2 = mj_name2id(this->model, mjtObj::mjOBJ_SENSOR, "thigh2_spos");
	motor1 = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, "thigh1_a");
	motor2 = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, "thigh2_a");

	radiusController.setPose(2, 1);
}

void LegControllerCPG::step() {
	double angle1 = data->sensordata[sensor1];
	double angle2 = data->sensordata[sensor2];
	double angleAvg = (angle1 - angle2) / 2;

	double expectedR = 0.1 * sin(angleAvg) + 0.2 * sin(acos(cos(angleAvg) / 2));
	double totalT = (angle1 + angle2);

	if (radiusController.getValue() == 0)
		radiusController.step(data->time - timePrev, 0.5);
	else
		radiusController.step(data->time - timePrev);

	timePrev = data->time;

	double pow = radiusController.getValue();
	printf("%f - %f\n", expectedR, pow);

	//data->ctrl[motor1] = pow;
}