#include "SingleUnitController.h"
#include "../include/mujoco.h"
#include <iostream>
#include <string>
#include <math.h>
#include <random>
using std::cout;
using std::endl;


SingleUnitController::SingleUnitController(const char *f, const char * actuatorNames[], const char * jointNames[], const char * endeffectorNames[]) : ModelController(f) {
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

	double speed = 1;
	double alpha = 50;
	double beta = 100;
	double b = beta*1.5;
	double range = 0.14;
	double swing = 14 * speed;
	double stance = 10 * speed;

	this->pattern1 = new CPGNode(alpha, beta, range, swing, stance, -1.0, 1.0);
	this->pattern2 = new CPGNode(alpha, beta, range, swing, stance, 1.0, -1.0);
	this->pattern3 = new CPGNode(alpha, beta, range, swing, stance, 1.0, -1.0);
	this->pattern4 = new CPGNode(alpha, beta, range, swing, stance, -1.0, 1.0);

	output.open("./telemetry.csv", std::ofstream::out | std::ofstream::trunc);
	for(int i = 0; i < datapoints; i++){
		output << "qpos[" << i << "],";
	}
}

SingleUnitController::~SingleUnitController() {
	free(this);
}

void SingleUnitController::step() {

	mj_kinematics(model, data);

	double dt = data->time - t;
	t = data->time;

	//double pitch = data->qpos[0]; //pitch angle of body (needed to keep legs pointed at ground)
	//double stoop = data->qpos[1]; //height of body from ground
	//double roll = data->qpos[2]; //roll angle of body

	pattern1->step(dt);
	pattern2->step(dt);
	pattern3->step(dt);
	pattern4->step(dt);

	double angle1 = PI/2 - pattern1->getAngle();
	double angle2 = PI/2 - pattern2->getAngle();
	double angle3 = PI/2 - pattern3->getAngle();
	double angle4 = PI/2 - pattern4->getAngle();

	backRight->setAngle(angle1);
	backLeft->setAngle(angle2);
	frontRight->setAngle(angle3);
	frontLeft->setAngle(angle4);

	backRight->setLength(pattern1->getLength());
	backLeft->setLength(pattern2->getLength());
	frontRight->setLength(pattern3->getLength());
	frontLeft->setLength(pattern4->getLength());

	backRight->setPgain(50);
	backLeft->setPgain(50);
	frontRight->setPgain(50);
	frontLeft->setPgain(50);

	backRight->step(data, model);
	backLeft->step(data, model);
	frontRight->step(data, model);
	frontLeft->step(data, model);

	int max = 15000;
	if(tick < max){
		for(int i = 0; i < datapoints; i++){
			output << data->qpos[i] << ",";
		}
		output << "\n";
	}else if(tick == max){
		output.close();
		cout << "Telemetry file written.\n";
	}
	tick++;

}
void SingleUnitController::keyboardCallback(GLFWwindow*, int key, int, int act, int mod) {
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
