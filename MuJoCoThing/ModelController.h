#ifndef ModelController_h
#define ModelController_h

#include "include\mujoco.h"

class ModelController {
public:
	virtual void step(mjModel*, mjData*) = 0;
	void setModelFile(const char *);
	const char * getModelFile();
private:
	const char * file;
};

#endif