#ifndef ModelController_h
#define ModelController_h

#include "include\mujoco.h"

//Base controller class does nothing in the step function
class ModelController {
public:
	ModelController(const char *);
	virtual void step(mjModel*, mjData*);
	void setModelFile(const char *);
	const char * getModelFile();
private:
	const char * file;
};

#endif