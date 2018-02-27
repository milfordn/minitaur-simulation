#ifndef CPGCTRL_H
#define CPGCTRL_H

#include "ModelController.h"
#include "CPGNode.h"

class LegControllerCPG : public ModelController {
public:
	LegControllerCPG(char * f);
	void step() override;
private:
	CPGNode radiusController;
	int sensor1, sensor2, motor1, motor2;
	mjtNum timePrev;
};

#endif