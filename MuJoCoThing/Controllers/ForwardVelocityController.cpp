#include "ForwardVelocityController.h"
#include "../include/mujoco.h"
#include <iostream>
#include <string>
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

	cout << m1AngleID << ", " << m2AngleID << ", " << m3AngleID << ", " << m4AngleID << ", " << m5AngleID << ", " << m6AngleID << ", " << m7AngleID << ", " << m8AngleID << endl;
	cout << end1 << ", " << end2 << ", " << end3 << ", " << end4 << endl;

	this->frontLeft = new LegPositionController(m1, m2, m1AngleID, m2AngleID, end1);
	this->frontRight = new LegPositionController(m3, m4, m3AngleID, m4AngleID, end2);
	this->backLeft = new LegPositionController(m5, m6, m5AngleID, m6AngleID, end3);
	this->backRight = new LegPositionController(m7, m8, m7AngleID, m8AngleID, end4);

	double pP = 0.1;
	double pI = 0.0;
	double pD = 0.00001;
	double max = 10;

	double rP = 0.5;
	double rI = 0.0;
	double rD = 0.00001;

	this->p = new pid(pP, pI, pD); //High-level PID controller for controlling pitch
	this->r = new pid(rP, rI, rD); //High-level PID controller for controlling roll
	p->limitOutput(-max, max);
	r->limitOutput(-max, max);
	speed = 3;

	pair1Theta = 1.57;
	pair2Theta = 1.57;
	pair1Speed = speed;
	pair2Speed = speed;

	output.open("./telemetry.csv", std::ofstream::out | std::ofstream::trunc);
	for(int i = 0; i < datapoints; i++){
		output << "qpos[" << i << "],";
	}
	cout << "\n";
	t = data->time;
	tick = 0;
}

ForwardVelocityController::~ForwardVelocityController() {
	free(this);
}

//theta = sin(sx - (1/2)sin(sx-P) - P)
double ForwardVelocityController::calculateTheta(double speed, double t, double offset, double bias){
	return PI/2 - (PI/8)*sin(speed*(t) - (0.5)*sin(speed*t - offset) - offset);
}

//L = (cos(sx - P) - 1)*cos(-sin(sx - P) + sx - P) - h
double ForwardVelocityController::calculateLength(double speed, double t, double offset, double height){
	return 0.09*(((cos(speed*t - offset)) - 1) * cos(-sin(speed*t - offset) + speed*t - offset)) - height;
}

//T = -(2/5) * ((-cos(sx - P) - 1)*cos(-sin(sx - P) + sx - P) - (1/2))
double ForwardVelocityController::calculateStiffness(double speed, double t, double offset){
	return -(.4) * ((-(cos(speed*t - offset)) - 1) * cos(-sin(speed*t - offset) + speed*t - offset) - 0.5) * 100;
}
void ForwardVelocityController::step() {

	mj_kinematics(model, data); //Calculate kinematics

	double t_delta = data->time - t;
	t = data->time;

	double height1 = .5;
	double height2 = .5;
	double height3 = .5;
	double height4 = .5;

	double pitch = data->qpos[0]; //pitch angle of body (needed to keep legs pointed at ground)
	double stoop = data->qpos[1]; //height of body from ground
	double roll = data->qpos[2]; //roll angle of body

	double interval = PI;
	double offset1 = 1 * interval;
	double offset2 = 0 * interval;
	double offset3 = 0 * interval;
	double offset4 = 1 * interval;

	double pitchControl = p->calculateOutput(t, 0.0, pitch);
	double rollControl = r->calculateOutput(t, 0.0, roll);

	//offset1 -= pitchControl + rollControl;
	//offset2 += pitchControl - rollControl;
	//offset3 += pitchControl + rollControl;
	//offset4 -= pitchControl - rollControl;
	//cout << pitchControl << " from " << pitch << ", and " << rollControl << " from " << roll << endl;
	//cout << data->qpos[0] << ", " << data->qpos[1] << ", " << data->qpos[2] << ", " << data->qpos[3] << ", " << data->qpos[4] << endl;

	//If backright is on the ground

	double speed1 = pair1Speed; //Back right
	double speed2 = pair2Speed; //Back left
	double speed3 = pair1Speed; //Front right
	double speed4 = pair2Speed; //Front left

	double leg1Theta = calculateTheta(speed1, t, offset1, pitch);
	double leg2Theta = calculateTheta(speed2, t, offset2, pitch);
	double leg3Theta = calculateTheta(speed3, t, offset3, pitch);
	double leg4Theta = calculateTheta(speed4, t, offset4, pitch);

	double leg1Length = calculateLength(speed1, t, offset1, height1);
	double leg2Length = calculateLength(speed2, t, offset2, height2);
	double leg3Length = calculateLength(speed3, t, offset3, height3);
	double leg4Length = calculateLength(speed4, t, offset4, height4);

	double leg1Stiffness = calculateStiffness(speed1, t, offset1);
	double leg2Stiffness = calculateStiffness(speed2, t, offset2);
	double leg3Stiffness = calculateStiffness(speed3, t, offset3);
	double leg4Stiffness = calculateStiffness(speed4, t, offset4);

	//cout << "front left: " << speed1 << ", " << leg1Theta << ", back right: " << speed3 << ", " << leg3Theta << endl;
	if(t < 0.2){
		leg1Theta = PI/2;
		leg2Theta = PI/2;
		leg3Theta = PI/2;
		leg4Theta = PI/2;
		leg1Length = 0;
		leg2Length = 0;
		leg3Length = 0;
		leg4Length = 0;
		offset1 = 0;
		offset2 = 0;
		offset3 = 0;
		offset4 = 0;
	}
	int max = 10000;
	if(tick < max){
		for(int i = 0; i < datapoints; i++){
			output << data->qpos[i] << ",";
		}
		output << "\n";
	}else if(tick == max){
		output.close();
		cout << "Telemetry file written.\n";
	}
	backRight->setAngle(leg1Theta);
	backLeft->setAngle(leg2Theta);
	frontRight->setAngle(leg3Theta);
	frontLeft->setAngle(leg4Theta);

	backRight->setLength(leg1Length);
	backLeft->setLength(leg2Length);
	frontRight->setLength(leg3Length);
	frontLeft->setLength(leg4Length);

	backRight->setPgain(leg1Stiffness);
	backLeft->setPgain(leg2Stiffness);
	frontRight->setPgain(leg3Stiffness);
	frontLeft->setPgain(leg4Stiffness);

	backRight->step(data, model);
	backLeft->step(data, model);
	frontRight->step(data, model);
	frontLeft->step(data, model);
	if(t > 2) tick++;
}
void ForwardVelocityController::keyboardCallback(GLFWwindow*, int key, int, int act, int mod) {
	/*cout << pair1Speed << endl;
	if(act != GLFW_PRESS) return;
	if(key == GLFW_KEY_W){
		pair1Speed += 0.00125;
		pair2Speed += 0.00125;
	}
	if(key == GLFW_KEY_S){
		pair1Speed -= 0.00125;
		pair2Speed -= 0.00125;
	}
	if(key == GLFW_KEY_A){
	}
	if(key == GLFW_KEY_D){
	}*/
}
