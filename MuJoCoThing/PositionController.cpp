#include "PositionController.h"
#include "include/mujoco.h"
#include <iostream>
using std::cout;
using std::endl;

PositionController::PositionController(const char *f, char * actuatorNames[], int size) : ModelController(f) {
	this->m1 = new pid(100, 0.0, 10.0); //allocate memory for PID controller
	this->m2 = new pid(100, 0.0, 10.0);
	this->actuatorIDs = new int[size]; //allocate required amount of memory to store actuator IDs
	
	//Use mj provided method for obtaining actuator IDs
	for (int i = 0; i < size; i++) { 
		this->actuatorIDs[i] = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, actuatorNames[i]);
		if(actuatorIDs[i] == -1){
			cout << "ACTUATOR NOT FOUND ERROR" << endl;
		}
	}
	this->size = size;
	this->tick = 0;
}

PositionController::~PositionController() {
	free(this);
}

void PositionController::step() {
	double output_1;
	double output_2;
	
	double m1_pos = data->qpos[actuatorIDs[0]];
	double m2_pos = data->qpos[actuatorIDs[1]];
	if(tick < 1000){
		output_1 = 50;
		output_2 = -50;
	}else{
		output_1 = 0;
		output_2 = 0;
	}
	/*double output;
	double m1_pos = data->qpos[1];
	if(data->qvel[0] > 0){ //If the velocity vectory isn't negative (not falling)
		output = m1->calculateOutput(tick, 0, data->qpos[1]); //Calculate the PID output from the motor angle
	}else{
		output = -4; //put legs back into position to jump
	}*/
	//Set the actuators to the PID output
	data->ctrl[actuatorIDs[0]] = output_1;
	data->ctrl[actuatorIDs[1]] = output_2;
	cout << m1_pos << ", " << m2_pos << endl;
	tick++;
	
}
