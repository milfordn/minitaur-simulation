#include "PositionController.h"
#include "include/mujoco.h"
#include <iostream>
#include <math.h>
using std::cout;
using std::endl;

PositionController::PositionController(const char *f, char * actuatorNames[], int size, char *endeffectorName) : ModelController(f) {
	this->angleController = new pid(2, 0.000, 10000.0); //allocate memory for PID controller
	this->lengthController = new pid(2, 0.00, 10000.0);
	this->m1 = new pid(5, 0.0, 10000);
	this->m2 = new pid(5, 0.0, 10000);
	this->actuatorIDs = new int[size]; //allocate required amount of memory to store actuator IDs
	
	//Use mj provided method for obtaining actuator IDs
	for (int i = 0; i < size; i++) { 
		this->actuatorIDs[i] = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, actuatorNames[i]);
		if(actuatorIDs[i] == -1){
			cout << "ACTUATOR NOT FOUND ERROR" << endl;
		}
	}
	this->endeffectorID = mj_name2id(this->model, mjtObj::mjOBJ_SITE, endeffectorName);
	cout << endeffectorID << endl;
	this->size = size;
	this->tick = 0;
}

PositionController::~PositionController() {
	free(this);
}

void PositionController::step() {
	mj_kinematics(model, data);
	
	if(tick < 20000){
		
	}else if(tick < 40000){
		setposX = -0.6;
		setposY = -0.6;
	}else if(tick < 60000){
		setposX = 1;
		setposY = 0;
	}else if(tick < 80000){
		setposX = -0.65;
		setposY = -.2;
	}else if(tick < 100000){
		setposX = 0;
		setposY = 1;
	}
	else if(tick < 120000){
		setposX = .77;
		setposY = .15;
	}else{
		setposX = 0;
		setposY = -0.5;
	}
	
	const double L1 = 0.5; //Length of thigh
	const double L2 = 1; //Length of shin

	
	double x = data->site_xpos[0]; //x coord of end effector
	double z = data->site_xpos[2] - 2; //z coord of end effector
	
	double desiredTheta = atan2(setposY, setposX); //Since the motors are at x,y (0, 0)
	double desiredLength = sqrt(setposY*setposY + setposX*setposX);
	
	double currentTheta = (data->qpos[0] + data->qpos[1])/2; 
	double currentLength = sqrt(x*x + z*z);
	
	double m1pos = desiredTheta + acos((desiredLength + L1*L1 - L2*L2)/(2 * L1 * desiredLength));
	double m2pos = desiredTheta - acos((desiredLength + L1*L1 - L2*L2)/(2 * L1 * desiredLength));
	
	double output_1;
	double output_2;
	
	//METHOD 1
	
	output_1 = m1->calculateOutput(tick, m1pos, data->qpos[0]);
	output_2 = m2->calculateOutput(tick, m2pos, data->qpos[2]);
	data->ctrl[0] = output_1;
	data->ctrl[1] = output_2;
	
	//METHOD 2
	/*
	output_1 = angleController->calculateOutput(tick, desiredTheta, currentTheta);
	output_2 = lengthController->calculateOutput(tick, desiredLength, currentLength);
	data->ctrl[0] = output_1 - output_2;
	data->ctrl[1] = output_1 + output_2;
	*/
	//cout << "DESIRED ANGLE: " << desiredTheta << " CURRENT ANGLE: " << currentTheta<< ", DESIRED LENGTH: " << desiredLength << ", CURRENT LENGTH: " << currentLength << endl; 
	//cout << m1pos << ", " << m2pos << endl;
	tick++;
	
}
