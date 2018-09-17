#include "ModelController.h"

ModelController::ModelController(const char * f) {
	setModelFile(f);

	char error[1000];
	this->model = mj_loadXML(f, NULL, error, sizeof(error));
	if(!this->model){
		mju_error_s("Couldn't load model: %s", error);
	}
	
	this->data = mj_makeData(model);
}

ModelController::ModelController(mjModel * m, mjData * d) {
	this->model = m;
	this->data = d;
	setModelFile("");
}

void ModelController::setModelFile(const char * f) {
	file = f;
}

const char * ModelController::getModelFile() {
	return file;
}

mjModel * ModelController::getModel() {
	return model;
}

mjData * ModelController::getData() {
	return data;
}

void ModelController::step() {}
void ModelController::keyboardCallback(GLFWwindow *, int, int, int, int) {}