#include "LegControllerCPG.h"
#include <cstdio>

LegControllerCPG::LegControllerCPG(char * f)
	: ModelController(f),
	ctrlR(-0.65, -0.05, -0.05), ctrlT(0.25, 0, 0.05) {
	sensor1 = mj_name2id(this->model, mjtObj::mjOBJ_SENSOR, "thigh1_spos");
	sensor2 = mj_name2id(this->model, mjtObj::mjOBJ_SENSOR, "thigh2_spos");
	motor1 = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, "thigh1_a");
	motor2 = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, "thigh2_a");
	sensorTouch = mj_name2id(this->model, mjtObj::mjOBJ_SENSOR, "foot_stouch");

	patternGenerator = new CPGNode(1, 50, mjPI / 4);
	patternGenerator->setPose(8, 4);
	patternGenerator->step(0, 0.001);
}

LegControllerCPG::LegControllerCPG(mjModel * m, mjData * d, char * s1, char * a1, char * s2, char * a2, CPGNode * cpg) 
	: ModelController(m, d), ctrlR(-0.65, -0.05, -0.05), ctrlT(0.25, 0, 0.05) {
	sensor1 = mj_name2id(this->model, mjtObj::mjOBJ_SENSOR, s1);
	sensor2 = mj_name2id(this->model, mjtObj::mjOBJ_SENSOR, s2);
	motor1 = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, a1);
	motor2 = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, a1);

	patternGenerator = cpg;
}

void LegControllerCPG::step() {
	double angle1 = data->sensordata[sensor1];
	double angle2 = data->sensordata[sensor2];
	double angleAvg = (angle1 - angle2) / 2;
	double s = data->sensordata[sensorTouch];

	double expectedR = 0.1 * sin(angleAvg) + 0.2 * sin(acos(cos(angleAvg) / 2));
	double totalT = (angle1 + angle2);

	patternGenerator->setFeedback(s - 0.1);
	patternGenerator->step(data->time - timePrev);
	timePrev = data->time;

	double y = patternGenerator->getValueY();
	double x = patternGenerator->getValueX();

	double setT = x;
	double setR = y > 0 ? 0.275 : 0.125;

	double dr = ctrlR.calculateOutput(data->time, setR, expectedR);
	double dt = ctrlT.calculateOutput(data->time, setT, totalT);

	data->ctrl[motor1] = dt - dr;
	data->ctrl[motor2] = dt + dr;

	//printf("%f -> %f | %f -> %f\n", expectedR, setR, totalT, setT);
	//printf("%f\n", data->sensordata[sensorTouch]);
}