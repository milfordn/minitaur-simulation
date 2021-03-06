#include "LegControllerCPG.h"
#include <cstdio>

LegControllerCPG::LegControllerCPG(char * f)
	: ModelController(f), patternGenerator(1, 50, mjPI / 2.75, 1.75, 12, 0, 0.001),
	anglectrl1(0, 0, 0, 0, 0), anglectrl2(0, 0, 0, 0, 0) {
	sensor1 = mj_name2id(this->model, mjtObj::mjOBJ_SENSOR, "thigh1_spos");
	sensor2 = mj_name2id(this->model, mjtObj::mjOBJ_SENSOR, "thigh2_spos");
	motor1 = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, "thigh1_a");
	motor2 = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, "thigh2_a");
	//sensorTouch = mj_name2id(this->model, mjtObj::mjOBJ_SENSOR, "foot_stouch");
}

LegControllerCPG::LegControllerCPG(mjModel * m, mjData * d, char * s1, char * a1, char * s2, char * a2, CPGNode * cpg) 
	: ModelController(m, d), ctrlR(5, 0, 0.2), ctrlT(0.75, 0, 0.05) {
	sensor1 = mj_name2id(this->model, mjtObj::mjOBJ_SENSOR, s1);
	sensor2 = mj_name2id(this->model, mjtObj::mjOBJ_SENSOR, s2);
	motor1 = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, a1);
	motor2 = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, a2);

}

void LegControllerCPG::step() {
	double angle1 = data->sensordata[sensor1] + mjPI / 2;
	double angle2 = data->sensordata[sensor2] - mjPI / 2;

	double measuredT = (angle1 + angle2);
	double angleCorrected = (angle1 - measuredT);

	double measuredR = 0.1 * sin(angleCorrected) + 0.2 * sin(acos(cos(angleCorrected) / 2));

	patternGenerator->step(data->time - timePrev);

	double y = patternGenerator->getLength();
	double x = -patternGenerator->getAngle();

	double setT = x;
	double setR = y > 0 ? 0.125 : 0.25;

	double dr = ctrlR.calculateOutput(data->time, setR, measuredR);
	double dt = ctrlT.calculateOutput(data->time, setT, measuredT);

	data->ctrl[motor1] = dt + dr;
	data->ctrl[motor2] = dt - dr;

	//printf("%f -> %f | %f -> %f\n", setR - measuredR, dr, setT - totalT, dt);
	//printf("%f -> %f | %f -> %f\n", measuredR, setR, measuredT, setT);
	//printf("%f\n", measuredR);
}
