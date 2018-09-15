#include "Controller.h"
#include <vector>
#include <string>
//using std::string;
//using std::vector;

void Controller::setSensorRef(unordered_map<string, std::vector<double>> * sref)
{
	this->sensorRef = sref;
}

void Controller::setActuatorRef(unordered_map<string, double> * aref)
{
	this->actuatorRef = aref;
}
