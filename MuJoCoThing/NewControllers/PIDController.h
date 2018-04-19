#ifndef NPIDCTRL_H
#define NPIDCTRL_H

#include "../Controller.h"
#include "../pid.h"
class PIDController : public Controller {
public:
	PIDController();
	void step(double dt) override;
private:
	pid ctrlR, ctrlT;
	double setR, setT;
	double counter;
};

#endif