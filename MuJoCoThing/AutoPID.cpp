#include "AutoPID.h"

AutoPID::AutoPID(pid controller, int actuatorID, int sensorID) 
: controller(controller) {
	this->actuatorID = actuatorID;
	this->sensorID = sensorID;
}

AutoPID::AutoPID(double kp, double ki, double kd, int actuatorID, int sensorID) 
: controller(kp, ki, kd)
{
	this->actuatorID = actuatorID;
	this->sensorID = sensorID;
}

void AutoPID::run(mjData * d, double setpoint) {
	d->ctrl[actuatorID] = controller.calculateOutput(d->time, setpoint, d->sensordata[sensorID]);
}