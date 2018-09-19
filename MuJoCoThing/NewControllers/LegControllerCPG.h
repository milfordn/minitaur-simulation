#ifndef LEGCPGCTRL_H
#define LEGCPGCTRL_H

#include "../Controller.h"
#include "../CPGNode.h"
#include "../pid.h"
#include "./PIDController.h"

class LegControllerCPG : public Controller {
public:
	LegControllerCPG(char * s1, char * s2, char * m1, char * m2, CPGNode * controller);
	void setSensorRef(unordered_map<string, vector<double>> *) override;
	void setActuatorRef(unordered_map<string, double> *) override;
	void step(double) override;
private:
	CPGNode * radiusController;
	char *sensor1, *sensor2, *motor1, *motor2;
	PIDController innerCtrl;
};

#endif