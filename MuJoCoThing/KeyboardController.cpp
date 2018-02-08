#include "KeyboardController.h"
#include "include\mujoco.h"

KeyboardController::KeyboardController(const char *f, int keys[], char * actuatorNames[], double powers[], int size) 
: ModelController(f) {
	this->keys = (int *)malloc(sizeof(int) * size);
	this->actuatorIDs = (int *)malloc(sizeof(int) * size);
	this->powers = (double *)malloc(sizeof(double) * size);

	for (int i = 0; i < size; i++) {
		this->keys[i] = keys[i];
		this->powers[i] = powers[i];
		this->actuatorIDs[i] = mj_name2id(this->model, mjtObj::mjOBJ_ACTUATOR, actuatorNames[i]);
	}

	this->size = size;
}

KeyboardController::~KeyboardController() {
	free(this->keys);
	free(this->actuatorIDs);
	free(this->powers);
}

void KeyboardController::step() {
	for (int i = 0; i < size; i++) {
		if (this->keys[i] == lastKey)
			data->ctrl[this->actuatorIDs[i]] = this->powers[i];
		else
			data->ctrl[this->actuatorIDs[i]] = 0;
	}
}

void KeyboardController::keyboardCallback(GLFWwindow *, int key, int, int act, int) {
	this->lastKey = (act == GLFW_RELEASE) ? -1 : key;
}