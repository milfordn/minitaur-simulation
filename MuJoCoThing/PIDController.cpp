#include "PIDController.h"
#include "include/mujoco.h"
#include <iostream>

PIDController::PIDController(const char *f, char * actuatorNames[], int size) : ModelController(f) {
	this->actuatorIDs = new int[size]; //allocate required amount of memory to store actuator IDs
	this->controller = new pid(100, 0.0, 10.0); //allocate memory for PID controller
	this->tick = 0;
	//Use mj provided method for obtaining actuator IDs
	for (int i = 0; i < size; i++) { 
		this->actuatorIDs[i] = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, actuatorNames[i]);
		if(actuatorIDs[i] == -1){
			std::cout << "ACTUATOR NOT FOUND ERROR" << std::endl;
		}
	}
	this->size = size;
}

PIDController::~PIDController() {
	free(this->actuatorIDs);
}

void PIDController::step() {
	double output;
	if(data->qvel[0] > 0){ //If the position of the root body is at height -2, and the velocity vectory isn't negative (not falling)
		output = controller->calculateOutput(tick, 0, data->qpos[1]); //Calculate the PID output from the motor angle
	}else{
		output = -4; //put legs back into position to jump
	}
	//Set the actuators to the PID output
	data->ctrl[this->actuatorIDs[0]] = -output;
	data->ctrl[this->actuatorIDs[1]] = output;
	tick++;
	
}
