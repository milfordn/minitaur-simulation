#include "ModelController.h"

ModelController::ModelController(const char * f) {
	setModelFile(f);

	char error[1000];

	this->model = mj_loadXML(f, NULL, error, sizeof(error));
	this->data = mj_makeData(model);
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