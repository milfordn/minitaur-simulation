#ifndef KEYCTRL_H
#define KEYCTRL_H

#include "ModelController.h"
#include "include\glfw3.h"

class KeyboardController : public ModelController {
public :
	KeyboardController(const char *, int [], char * [], double [], int size);
	~KeyboardController();
	void step() override;
	void keyboardCallback(GLFWwindow *, int, int, int, int) override;
private:
	int size;
	int * keys;
	int * actuatorIDs;
	double * powers;

	int lastKey;
};

#endif
