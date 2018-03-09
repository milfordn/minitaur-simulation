#ifndef CPGCTRL_H
#define CPGCTRL_H

#include "ModelController.h"
#include "CPGNode.h"
#include "../AutoPID.h"

class LegControllerCPG : public ModelController {
public:
	LegControllerCPG(char * f);
	LegControllerCPG(mjModel * m, mjData * d, char * s1, char * a1, char * s2, char * a2, CPGNode * cpg);
	void step() override;
private:
	CPGNode * patternGenerator;
	int sensor1, sensor2, sensorTouch, motor1, motor2;
	mjtNum timePrev;
	pid ctrlR, ctrlT;
};

#endif