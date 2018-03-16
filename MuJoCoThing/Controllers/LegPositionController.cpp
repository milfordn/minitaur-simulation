#include "LegPositionController.h"
#include "../include/mujoco.h"
#include <iostream>
#include <math.h>
#include <random>

using std::cout;
using std::endl;

//TODO - take four leg names as input, create subclass to handle individual leg position

LegPositionController::LegPositionController(int m1ID, int m2ID, int m1JointID, int m2JointID, int endID){
	//allocate memory for PID controllers
	//this->angleController = new pid(15, 0.000, 5000.0);
	//this->lengthController = new pid(15, 0.00, 5000.0);
	this->m1 = new pid(0.3, 0.0, 10);
	this->m2 = new pid(0.3, 0.0, 10);

	this->m1ID = m1ID;
	this->m2ID = m2ID;
	this->m1JointID = m1JointID;
	this->m2JointID = m2JointID;
	this->endID = endID;

	tick = 0;
}

LegPositionController::~LegPositionController() {
	free(this);
}

void LegPositionController::step(struct _mjData* data, struct _mjModel* model) {
	const double L1 = 0.1;
	const double L2 = 0.2;

	double currentm1pos = data->qpos[model->jnt_qposadr[m1JointID]];
	double currentm2pos = data->qpos[model->jnt_qposadr[m2JointID]];

	double currentTheta = (currentm1pos + currentm2pos)/2; //The current angle of the leg
	double currentLength = L1 * sin(currentTheta - currentm1pos) + sqrt(L2*L2 - L1*L1*cos(currentTheta - currentm1pos)*cos(currentTheta - currentm1pos)); //The current distance from the motors to the end effector


	double m1pos = desiredAngle - asin((desiredLength*desiredLength + L1*L1 - L2*L2)/(2 * L1 * desiredLength)); //desired pos of motor1
	double m2pos = desiredAngle + asin((desiredLength*desiredLength + L1*L1 - L2*L2)/(2 * L1 * desiredLength)); //desired pos of motor2

	//METHOD 1
	double output_1 = m1->calculateOutput(tick, m1pos, currentm1pos);
	double output_2 = m2->calculateOutput(tick, m2pos, currentm2pos);

	if(active){
		data->ctrl[m1ID] = output_1;
		data->ctrl[m2ID] = output_2;
	}else{
		data->ctrl[m1ID] = 0;
		data->ctrl[m2ID] = 0;
	}

	//METHOD 2
	/*
	double output_1 = angleController->calculateOutput(tick, desiredTheta, currentTheta);
	double output_2 = lengthController->calculateOutput(tick, desiredLength, currentLength);
	data->ctrl[0] = output_1 + output_2;
	data->ctrl[1] = output_1 - output_2;
	*/
	tick++;

}
void LegPositionController::setPgain(double p){
	this->m1->setPgain(p);
	this->m2->setPgain(p);
}
void LegPositionController::release(){
	active = false;
}
void LegPositionController::setLength(double length){
	active = true;
	desiredLength = length;
}
void LegPositionController::setAngle(double angle){
	active = true;
	desiredAngle = angle;
}
