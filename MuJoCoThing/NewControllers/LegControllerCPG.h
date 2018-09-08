#ifndef CPGCTRL_H
#define CPGCTRL_H

#include "../Controller.h"
#include "../CPGNode.h"
#include "../pid.h"

class LegControllerCPG : public Controller {
public:
	LegControllerCPG(char * s1, char * s2, char * m1, char * m2);
	void step(double) override;
private:
	CPGNode radiusController;
	char *sensor1, *sensor2, *motor1, *motor2;
	pid anglectrl1, anglectrl2;
};

#endif