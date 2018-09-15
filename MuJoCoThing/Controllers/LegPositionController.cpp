#include "LegPositionController.h"
#include "../include/mujoco.h"
#include <iostream>
#include <math.h>
#include <random>

using std::cout;
using std::endl;

LegPositionController::LegPositionController(int m1ID, int m2ID, int m1JointID, int m2JointID, int endID){
	//allocate memory for PID controllers
	//this->angleController = new pid(15, 0.000, 5000.0);
	//this->lengthController = new pid(15, 0.00, 5000.0);
	this->m1 = new pid(100, 0.0, 1);
	this->m2 = new pid(100, 0.0, 1);

	this->m1ID = m1ID;
	this->m2ID = m2ID;
	this->m1JointID = m1JointID;
	this->m2JointID = m2JointID;
	this->endID = endID;
	this->currentLength = 0.191;

	tick = 0;
}

LegPositionController::~LegPositionController() {
	free(this);
}

void LegPositionController::step(struct _mjData* data, struct _mjModel* model) {
	L1 = 0.1;
	L2 = 0.2;

	double currentm1pos = data->qpos[model->jnt_qposadr[m1JointID]];
	double currentm2pos = data->qpos[model->jnt_qposadr[m2JointID]];

	currentAngle = (currentm1pos + currentm2pos)/2; //The current angle of the leg
	currentLength = L1 * sin(currentAngle - currentm1pos) + sqrt(L2*L2 - L1*L1*cos(currentAngle - currentm1pos)*cos(currentAngle - currentm1pos)); //The current distance from the motors to the end effector


	double m1pos = desiredAngle - asin((desiredLength*desiredLength + L1*L1 - L2*L2)/(2 * L1 * desiredLength)); //desired pos of motor1
	double m2pos = desiredAngle + asin((desiredLength*desiredLength + L1*L1 - L2*L2)/(2 * L1 * desiredLength)); //desired pos of motor2
	//METHOD 1
	double output_1 = m1->calculateOutput(data->time, m1pos, currentm1pos);
	double output_2 = m2->calculateOutput(data->time, m2pos, currentm2pos);
	//cout << "currentm1: " << currentm1pos << ", desired: " << m1pos << "output: " << output_1 << endl;
	if(active){
		data->ctrl[m1ID] = output_1;
		data->ctrl[m2ID] = output_2;
	}else{
		data->ctrl[m1ID] = 0;
		data->ctrl[m2ID] = 0;
	}
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
	if(length > 1) length = 1;
	if(length < -1) length = -1;
	desiredLength = 0.191 + 0.09*length;
}
void LegPositionController::setAngle(double angle){
	active = true;
	desiredAngle = angle;
}
double LegPositionController::getLength(){
	double input_range = 2*L1;
	double output_range = 2.0;
	return (currentLength - L1)*output_range / input_range + -1;
}
double LegPositionController::getAngle(){
	return currentAngle;
}
