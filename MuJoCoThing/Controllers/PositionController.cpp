#include "PositionController.h"
#include "../include/mujoco.h"
#include <iostream>
#include <math.h>
#include <random>

using std::cout;
using std::endl;


PositionController::PositionController(const char *f, const char * actuatorNames[], const char * jointNames[], const char * endeffectorNames[]) : ModelController(f) {
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

	this->tick = 0;
}

PositionController::~PositionController() {
	//free(this);
	free(frontLeft);
	free(frontRight);
	free(backLeft);
	free(backRight);
}

void PositionController::step() {
	mj_kinematics(model, data); //Calculate kinematics
	double desiredLength = .102;
	double desiredAngle = 1.57;
	
	double activeP = 1.5;
	double inactiveP = 0.28;
	
	frontLeft->setAngle(desiredAngle);
	frontRight->setAngle(desiredAngle);
	backLeft->setAngle(desiredAngle);
	backRight->setAngle(desiredAngle);
	
	if(data->qvel[0] < 0){
		desiredLength = 0.28;
		frontLeft->setPgain(inactiveP);
		frontRight->setPgain(inactiveP);
		backLeft->setPgain(inactiveP);
		backRight->setPgain(inactiveP);
	}else{
		frontLeft->setPgain(activeP);
		frontRight->setPgain(activeP);
		backLeft->setPgain(activeP);
		backRight->setPgain(activeP);
	}
	
	frontLeft->setLength(desiredLength);
	frontRight->setLength(desiredLength);
	backLeft->setLength(desiredLength);
	backRight->setLength(desiredLength);
	
	frontLeft->step(data, model);
	frontRight->step(data, model);
	backLeft->step(data, model);
	backRight->step(data, model);
	
	tick++;
}



