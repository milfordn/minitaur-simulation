#include "LegControllerCPG.h"
#include <cstdio>

LegControllerCPG::LegControllerCPG(char * f)
	: ModelController(f), patternGenerator(1, 50, mjPI / 3), 
	ctrlR(-0.65, -0.05, -0.05), ctrlT(0.2, 0, 0.05) {
	sensor1 = mj_name2id(this->model, mjtObj::mjOBJ_SENSOR, "thigh1_spos");
	sensor2 = mj_name2id(this->model, mjtObj::mjOBJ_SENSOR, "thigh2_spos");
	motor1 = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, "thigh1_a");
	motor2 = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, "thigh2_a");

	patternGenerator.setPose(12, 4);
	patternGenerator.step(0, 0.001);
}

void LegControllerCPG::step() {
	double angle1 = data->sensordata[sensor1];
	double angle2 = data->sensordata[sensor2];
	double angleAvg = (angle1 - angle2) / 2;

	double expectedR = 0.1 * sin(angleAvg) + 0.2 * sin(acos(cos(angleAvg) / 2));
	double totalT = (angle1 + angle2);

	patternGenerator.step(data->time - timePrev);
	timePrev = data->time;

	double setT = patternGenerator.getValueX();
	double setR = patternGenerator.getValueY() > 0 ? 0.125 : 0.275;

	double dr = ctrlR.calculateOutput(data->time, setR, expectedR);
	double dt = ctrlT.calculateOutput(data->time, setT, totalT);

	data->ctrl[motor1] = dt - dr;
	data->ctrl[motor2] = dt + dr;

	printf("%f -> %f | %f -> %f\n", expectedR, setR, totalT, setT);
}