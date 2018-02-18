#ifndef MTCTRL_H
#define MTCTRL_H

#include "ModelController.h"
#include "AutoPID.h"

class MinitaurLegController : public ModelController {
public:
	MinitaurLegController(const char *);
	void step() override;
	void keyboardCallback(GLFWwindow *, int, int, int, int) override;
private:
	AutoPID ctrl1, ctrl2;
	double angle;
};

#endif
