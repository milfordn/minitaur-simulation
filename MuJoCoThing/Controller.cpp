#include "Controller.h"
#include <vector>
#include <string>

void Controller::setSensorRef(unordered_map<string, std::vector<double>> * sref)
{
	this->sensorRef = sref;
}

void Controller::setActuatorRef(unordered_map<string, double> * aref)
{
	this->actuatorRef = aref;
}
