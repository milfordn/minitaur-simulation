#include "ModelController.h"

void ModelController::setModelFile(const char * f) {
	file = f;
};

const char * ModelController::getModelFile() {
	return file;
}