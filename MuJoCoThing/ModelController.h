#ifndef ModelController_h
#define ModelController_h

#include "include/mujoco.h"
#include "include/glfw3.h"

//Base controller class does nothing in the step function
class ModelController {
public:
	ModelController(const char *);
	virtual void step();
	virtual void keyboardCallback(GLFWwindow *, int, int, int, int);

	void setModelFile(const char *);
	const char * getModelFile();
	mjModel * getModel();
	mjData * getData();
private:
	const char * file;
protected:
	mjModel * model;
	mjData * data;	
};

#endif