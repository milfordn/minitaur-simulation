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
}

ForwardVelocityController::~ForwardVelocityController() {
	free(this);
}

void ForwardVelocityController::step() {
	double PI = 3.14159265359;
	mj_kinematics(model, data); //Calculate kinematics
	//Back left and front right work together
	//Back right and front left work together
	//Difference in angle between two pairs -> goes to 0 as speed increases (bounding vs trotting)
	//Difference in angle of two legs inside a pair -> goes to max as speed increases (pairs switch from opposites to front/back)
		//Need maximum difference in leg angle?
	//leg1 = leg3
	//leg2 = leg4


	//If leg moving forward, length short
	//If leg moving backward, length long

	//.191 + .089*sin(angle)
	//Difference in length between two pairs

	pair1Theta += 0.001 * pair1Speed;
	pair2Theta += 0.001 * pair2Speed;

	if(pair1Theta > PI/2 + PI/6 || pair1Theta < PI/2 - PI/6) pair1Speed *= -1;

	if(pair2Theta > PI/2 + PI/6 || pair2Theta < PI/2 - PI/6) pair2Speed *= -1;

	double pair1Length = 0.191 + 0.018 * sin(pair1Theta) * pair1Speed;
	double pair2Length = 0.191 + 0.018 * sin(pair2Theta) * pair2Speed;

//	frontLeft->release();
//	frontRight->release();
//	backLeft->release();
//	backRight->release();

	frontLeft->setAngle(pair1Theta);
	frontRight->setAngle(pair2Theta);
	backLeft->setAngle(pair2Theta);
	backRight->setAngle(pair1Theta);

	frontLeft->setLength(pair1Length);
	frontRight->setLength(pair2Length);
	backLeft->setLength(pair2Length);
	backRight->setLength(pair1Length);

	frontLeft->step(data, model);
	frontRight->step(data, model);
	backLeft->step(data, model);
	backRight->step(data, model);


}
