#include "CustomSimulate.h"
#include "KeyboardController.h"
#include "PIDController.h"
#include "include/glfw3.h"
#include "ModelController.h"

int main(int argc, char ** argv) {
	mj_activate("mjkey.txt");
	//ModelController m("MinitaurLeg.xml");
	//char * names[2] = { (char*)"thigh1_a", (char*)"thigh2_a" };
	char * names[2] = { (char*)"motor_a", (char*)"motor_b" };
	int keys[2] = { GLFW_KEY_A, GLFW_KEY_D };
	double powers[2] = { -0.01, 0.01 };
	
	KeyboardController k(argv[1], keys, names, powers, 2);
	
	PIDController p(argv[1], names, 2);
	
	run(&p);

	return 0;
}