#include "PIDController.h"
#include "include/mujoco.h"

PIDController::PIDController(const char *f, char * actuatorNames[], int size) : ModelController(f) {
	this->actuatorIDs = (int *)malloc(sizeof(int) * size); //allocate required amount of memory to store actuator IDs
	this->controller = new pid(100, 0.0, 10.0); //allocate memory for PID controller

	//Use mj provided method for obtaining actuator IDs
	for (int i = 0; i < size; i++) { 
		this->actuatorIDs[i] = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, actuatorNames[i]);
	}
	this->size = size;

}

PIDController::~PIDController() {
	free(this->actuatorIDs);
}

void PIDController::step() {
	double output;
	if(data->qpos[0] < -2 && data->qvel[0] > 0){ //If the position of the root body is at height -2, and the velocity vectory isn't negative (not falling)
		output = controller->calculateOutput(data->qpos[1], 0); //Calculate the PID output
	}else{
		output = 0;
	}
	mj_step(model, data);
	
	//Set the actuators to the PID output
	data->ctrl[0] = -output; 
	data->ctrl[1gi] = output;
	
}
