#include "System.h"

void System::setSensorRef(unordered_map<string, std::vector<double>> * sref)
{
	this->sensorRef = sref;
}

void System::setActuatorRef(unordered_map<string, double> * aref)
{
	this->actuatorRef = aref;
}
