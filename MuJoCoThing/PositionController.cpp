#include "PositionController.h"
#include "include/mujoco.h"
#include <iostream>
#include <math.h>
#include <random>

using std::cout;
using std::endl;

//TODO - take four leg names as input, create subclass to handle individual leg position

PositionController::PositionController(const char *f, char * actuatorNames[], int size, char *endeffectorName) : ModelController(f) {
	//allocate memory for PID controllers
	this->angleController = new pid(15, 0.000, 5000.0); 
	this->lengthController = new pid(15, 0.00, 5000.0);
	this->m1 = new pid(25, 0.0, 50000);
	this->m2 = new pid(25, 0.0, 50000);
	this->actuatorIDs = new int[size];
	
	for (int i = 0; i < size; i++) { 
		this->actuatorIDs[i] = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, actuatorNames[i]);
	}
	this->endeffectorID = mj_name2id(this->model, mjtObj::mjOBJ_SITE, endeffectorName);
	this->size = size;
	this->tick = 0;
}

PositionController::~PositionController() {
	free(this);
}

void PositionController::step() {
	mj_kinematics(model, data); //Calculate kinematics so that site_xpos isn't 0
	
	//Every 20,000 ticks we pick a new random location that the leg has to orient itself to (physically impossible to be within 0.5 units of origin, so account for that special case)
	if(tick % 20000 == 0){
		double newx = ((double)rand()/(double)RAND_MAX) * 3 - 1.5;
		double newy = ((double)rand()/(double)RAND_MAX) * 3 - 1.5;
		while(!((newx > 0.5 || newx < -0.5) && (newy > 0.5 || newy < -0.5))){
			newx = ((double)rand()/(double)RAND_MAX) * 3 - 1.5;
			newy = ((double)rand()/(double)RAND_MAX) * 3 - 1.5;
		}
		setposX = newx;
		setposY = newy;
	}
	
	double x = data->site_xpos[endeffectorID]; //x coord of end effector
	double z = data->site_xpos[endeffectorID+2] - 2; //z coord of end effector
	
	double desiredTheta = atan2(setposY, setposX); //Since the motors are at x,y (0, 0)
	double desiredLength = sqrt(setposY*setposY + setposX*setposX);
	
	double currentTheta = (data->qpos[0] + data->qpos[2])/2; //The current angle of the leg
	double currentLength = sqrt(x*x + z*z); //The current distance from the motors to the end effector
	
	double m1pos = desiredTheta + acos((desiredLength + L1*L1 - L2*L2)/(2 * L1 * desiredLength)); //desired position of motor1
	double m2pos = desiredTheta - acos((desiredLength + L1*L1 - L2*L2)/(2 * L1 * desiredLength)); //desired position of motor2
	
	//METHOD 1
	double output_1 = m1->calculateOutput(tick, m1pos, data->qpos[0]);
	double output_2 = m2->calculateOutput(tick, m2pos, data->qpos[2]);
	data->ctrl[actuatorIDs[0]] = output_1;
	data->ctrl[actuatorIDs[1]] = output_2;
	
	//METHOD 2
	/*
	double output_1 = angleController->calculateOutput(tick, desiredTheta, currentTheta);
	double output_2 = lengthController->calculateOutput(tick, desiredLength, currentLength);
	data->ctrl[0] = output_1 + output_2;
	data->ctrl[1] = output_1 - output_2;
	*/
	tick++;
	
}
