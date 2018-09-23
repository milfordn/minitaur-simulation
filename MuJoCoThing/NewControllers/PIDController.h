#ifndef NPIDCTRL_H
#define NPIDCTRL_H

#include "../Controller.h"
#include "../pid.h"
class PIDController : public Controller {
public:
	PIDController(char * s1, char * s2, char * a1, char * a2);
	void setpoint(double radius, double angle);
	void setRadiusGains(double kp, double ki, double kd);
	void setAngleGains(double kp, double ki, double kd);
	void step(double dt) override;
private:
	pid ctrlR, ctrlT;
	double setR, setT;
	double counter;
	char * sensor1, * sensor2, * actuator1, * actuator2;
};

#endif
