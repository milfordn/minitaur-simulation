#include "ForwardVelocityController.h"
#include "../include/mujoco.h"
#include <iostream>
#include <math.h>
#include <random>

using std::cout;
using std::endl;


ForwardVelocityController::ForwardVelocityController(const char *f, const char * actuatorNames[], const char * jointNames[], const char * endeffectorNames[]) : ModelController(f) {
	//allocate memory for leg position controllers
	int m1 = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, actuatorNames[0]);
	int m2 = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, actuatorNames[1]);
	int m3 = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, actuatorNames[2]);
	int m4 = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, actuatorNames[3]);
	int m5 = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, actuatorNames[4]);
	int m6 = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, actuatorNames[5]);
	int m7 = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, actuatorNames[6]);
	int m8 = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, actuatorNames[7]);

	int m1AngleID = mj_name2id(this->model, mjtObj::mjOBJ_JOINT, jointNames[0]);
	int m2AngleID = mj_name2id(this->model, mjtObj::mjOBJ_JOINT, jointNames[1]);
	int m3AngleID = mj_name2id(this->model, mjtObj::mjOBJ_JOINT, jointNames[2]);
	int m4AngleID = mj_name2id(this->model, mjtObj::mjOBJ_JOINT, jointNames[3]);
	int m5AngleID = mj_name2id(this->model, mjtObj::mjOBJ_JOINT, jointNames[4]);
	int m6AngleID = mj_name2id(this->model, mjtObj::mjOBJ_JOINT, jointNames[5]);
	int m7AngleID = mj_name2id(this->model, mjtObj::mjOBJ_JOINT, jointNames[6]);
	int m8AngleID = mj_name2id(this->model, mjtObj::mjOBJ_JOINT, jointNames[7]);

	int end1 = mj_name2id(this->model, mjtObj::mjOBJ_SITE, endeffectorNames[0]);
	int end2 = mj_name2id(this->model, mjtObj::mjOBJ_SITE, endeffectorNames[1]);
	int end3 = mj_name2id(this->model, mjtObj::mjOBJ_SITE, endeffectorNames[2]);
	int end4 = mj_name2id(this->model, mjtObj::mjOBJ_SITE, endeffectorNames[3]);

	this->frontLeft = new LegPositionController(m1, m2, m1AngleID, m2AngleID, end1);
	this->frontRight = new LegPositionController(m3, m4, m3AngleID, m4AngleID, end2);
	this->backLeft = new LegPositionController(m5, m6, m5AngleID, m6AngleID, end3);
	this->backRight = new LegPositionController(m7, m8, m7AngleID, m8AngleID, end4);
	pair1Theta = 1.57;
	pair2Theta = 1.57;;
	pair1Speed = 1.0;
	pair2Speed = -1.0;
	t = data->time;
	tick = 0;
}

ForwardVelocityController::~ForwardVelocityController() {
	free(this);
}

void ForwardVelocityController::step() {
	double PI = 3.14159265359;
	mj_kinematics(model, data); //Calculate kinematics
	double speed = 5;
	double t_delta = data->time - t;
	t = data->time;
  //FUNCTION TO MATCH: THETA = sin(x - 1/2sin(x))

	double offset1 = 0;
	double offset2 = PI/2;
	double offset3 = PI;
	double offset4 = 3*PI/2;

	double height = PI;

	double leg1Theta = PI/2 - (PI/6)*sin(speed*(t) - (0.5)*sin(speed*t - offset1) - offset1);
	double leg2Theta = PI/2 - (PI/6)*sin(speed*(t) - (0.5)*sin(speed*t - offset2) - offset2);
	double leg3Theta = PI/2 - (PI/6)*sin(speed*(t) - (0.5)*sin(speed*t - offset3) - offset3);
	double leg4Theta = PI/2 - (PI/6)*sin(speed*(t) - (0.5)*sin(speed*t - offset4) - offset4);

	double leg1Length = 0.191 + 0.018 * (((cos(speed*t - offset1)) - 1) * cos(sin(speed*t - offset1) + speed*t - offset1) - height);
	double leg2Length = 0.191 + 0.018 * (((cos(speed*t - offset2)) - 1) * cos(sin(speed*t - offset2) + speed*t - offset2) - height);
	double leg3Length = 0.191 + 0.018 * (((cos(speed*t - offset3)) - 1) * cos(sin(speed*t - offset3) + speed*t - offset3) - height);
	double leg4Length = 0.191 + 0.018 * (((cos(speed*t - offset4)) - 1) * cos(sin(speed*t - offset4) + speed*t - offset4) - height);

	//double leg1Stiffness = -(cos(t+offset1) - 1)*cos(sin(t+offset1) -;
	double leg2Stiffness;
	double leg3Stiffness;
	double leg4Stiffness;

	frontLeft->setAngle(leg1Theta);
	frontRight->setAngle(leg2Theta);
	backLeft->setAngle(leg3Theta);
	backRight->setAngle(leg4Theta);

	frontLeft->setLength(leg1Length);
	frontRight->setLength(leg2Length);
	backLeft->setLength(leg3Length);
	backRight->setLength(leg4Length);
/*
	frontLeft->setPgain(leg1Stiffness);
	frontRight->setPgain(leg2Stiffness);
	backLeft->setPgain(leg3Stiffness);
	backRight->setPgain(leg4Stiffness);
*/
	frontLeft->step(data, model);
	frontRight->step(data, model);
	backLeft->step(data, model);
	backRight->step(data, model);
}
