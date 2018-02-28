#include "LegPositionController.h"
#include "include/mujoco.h"
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
	this->m1 = new pid(1, 0.0, 0.01);
	this->m2 = new pid(1, 0.0, 0.01);
	
	this->m1ID = m1ID;
	this->m2ID = m2ID;
	this->m1JointID = m1JointID;
	this->m2JointID = m2JointID;
	this->endID = endID;
	
	desiredLength = 8;
	desiredAngle = 1.57;
	
	tick = 0;
}

LegPositionController::~LegPositionController() {
	free(this);
}

void LegPositionController::step(struct _mjData* data, struct _mjModel* model) {
	//double x = data->site_xpos[endID]; //x coord of end effector
	//double z = data->site_xpos[endID+2] - 2;// - 2 //z coord of end effector
	if(!data){
		cout << "ERR" << endl;
	}
	double currentm1pos = data->qpos[model->jnt_qposadr[m1JointID]];
	double currentm2pos = data->qpos[model->jnt_qposadr[m2JointID]];
	
	double currentTheta = (currentm1pos + currentm2pos)/2; //The current angle of the leg
	double currentLength = (L1*L1 - L2*L2) / (cos(currentm1pos - currentTheta)*2*L1 - 1); //The current distance from the motors to the end effector
	
	double m1pos = desiredAngle + acos((desiredLength*desiredLength + L1*L1 - L2*L2)/(2 * L1 * desiredLength)); //desired pos of motor1
	double m2pos = desiredAngle - acos((desiredLength*desiredLength + L1*L1 - L2*L2)/(2 * L1 * desiredLength)); //desired pos of motor2
	
	//METHOD 1
	double output_1 = m1->calculateOutput(tick, m1pos, currentm1pos);
	double output_2 = m2->calculateOutput(tick, m2pos, currentm2pos);
	
	data->ctrl[m1ID] = output_1;
	data->ctrl[m2ID] = output_2;
	
	//cout << m1ID  << ", " << data->qpos[m1JointID] << ", " << data->ctrl[m2ID] << ", " << data->qpos[m2JointID] << endl;
	//cout << m1pos << " , " << m2pos << " . ";
	//METHOD c
	/*
	double output_1 = angleController->calculateOutput(tick, desiredTheta, currentTheta);
	double output_2 = lengthController->calculateOutput(tick, desiredLength, currentLength);
	data->ctrl[0] = output_1 + output_2;
	data->ctrl[1] = output_1 - output_2;
	*/
	tick++;
	
}
void LegPositionController::release(struct _mjData* data){
	data->ctrl[m1ID] = 0;
	data->ctrl[m2ID] = 0;
}
void LegPositionController::setLength(double length){
	desiredLength = length;
}
void LegPositionController::setAngle(double angle){
	desiredAngle = angle;
}