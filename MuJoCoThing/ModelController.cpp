#include "ModelController.h"

ModelController::ModelController(const char * f) {
	setModelFile(f);
}

void ModelController::setModelFile(const char * f) {
	file = f;
}

const char * ModelController::getModelFile() {
	return file;
}

void ModelController::step(mjModel * m, mjData * d) {}