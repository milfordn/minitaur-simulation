#include "Controller.h"

void Controller::setSensorRef(unordered_map<string, double> * sref)
{
	this->sensorRef = sref;
}

void Controller::setActuatorRef(unordered_map<string, double> * aref)
{
	this->actuatorRef = aref;
}
