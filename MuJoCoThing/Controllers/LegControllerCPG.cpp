#include "LegControllerCPG.h"
#include <cstdio>

LegControllerCPG::LegControllerCPG(char * f)
	: ModelController(f), radiusController(1, 50, mjPI / 2.75, 1.75, 12, 0, 0.001),
	anglectrl1(0, 0, 0, 0, 0), anglectrl2(0, 0, 0, 0, 0) {
	sensor1 = mj_name2id(this->model, mjtObj::mjOBJ_SENSOR, "thigh1_spos");
	sensor2 = mj_name2id(this->model, mjtObj::mjOBJ_SENSOR, "thigh2_spos");
	motor1 = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, "thigh1_a");
	motor2 = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, "thigh2_a");

	anglectrl1 = AutoPID(0.25, 0, 0.01, motor1, sensor1);
	anglectrl2 = AutoPID(0.25, 0, 0.01, motor2, sensor2);

}

void LegControllerCPG::step() {
	double angle1 = data->sensordata[sensor1];
	double angle2 = data->sensordata[sensor2];

	radiusController.step(data->time - timePrev);//, angle1 - mjPI / 3);
	double nextAngle1 = radiusController.getAngle() + mjPI * 0.05;
	timePrev = data->time;

	anglectrl1.run(data, nextAngle1);
	anglectrl2.run(data, -nextAngle1);

	printf("%f -> %f | %f -> %f\n", angle1, nextAngle1, angle2, -nextAngle1);
}
