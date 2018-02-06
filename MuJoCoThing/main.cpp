#include "CustomSimulate.h"
#include "KeyboardController.h"
#include "include\glfw3.h"
#include "ModelController.h"

int main(int argc, char ** argv) {
	mj_activate("mjkey.txt");
	//ModelController m("MinitaurLeg.xml");
	char * names[2] = { "thigh1_a", "thigh2_a" };
	int keys[2] = { GLFW_KEY_A, GLFW_KEY_D };
	double powers[2] = { -0.01, 0.01 };
	KeyboardController k("MinitaurLeg.xml", keys, names, powers, 2);

	run(&k);

	return 0;
}