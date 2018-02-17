#include "ModelController.h"
#include "include/glfw3.h"
#include "pid.h"

class PIDController : public ModelController {
public :
	PIDController(const char *, char * [], int size);
	~PIDController();
	void step() override;
private:
	int size;
	unsigned long tick;
	int * actuatorIDs;
	pid *controller;
};

