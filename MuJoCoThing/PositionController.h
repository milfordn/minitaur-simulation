#include "ModelController.h"
#include "pid.h"

class PositionController : public ModelController {
public :
	PositionController(const char *, char * [], int size, char *);
	~PositionController();
	void step() override;
private:
	unsigned long tick;
	int * actuatorIDs;
	int endeffectorID;
	int size;
	pid *m1; //Torque-level PID controller for controlling angle of motor 1
	pid *m2; //Torque-level PID controller for controlling angle of motor 2
	pid *angleController; //PID Controller for controlling relative angle between motor 1 and 2
	pid *lengthController; //PID Controller for controlling distance from origin to end effector
	double setposY = 1; //Initial vertical setpoint
	double setposX = 0.5; //Initial horizontal setpoint
	const double L1 = 0.5; //Length of thigh
	const double L2 = 1; //Length of shin
};

