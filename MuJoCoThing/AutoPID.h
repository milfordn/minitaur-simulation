#ifndef AUTOPID_H
#define AUTOPID_H

#include "pid.h"
#include "include/mujoco.h"

class AutoPID {
public:
	AutoPID(pid controller, int actuatorID, int sensorID);
	AutoPID(double kp, double ki, double kd, int actuatorID, int sensorID);
	void run(mjData * d, double setpoint);
private:
	pid controller;
	int actuatorID, sensorID;
};

#endif
