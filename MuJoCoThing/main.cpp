
#include "Mediator.h"
#include "Systems/MujocoSystem.h"
#include "Controller.h"
#include "NewControllers/PIDController.h"

int main(int argc, char ** argv) {
	mj_activate("mjkey.txt");
	//char * names[2] = { (char*)"motor_a", (char*)"motor_c" };
	//int keys[2] = { GLFW_KEY_A, GLFW_KEY_D };
	//double powers[2] = { -0.01, 0.01 };
	//KeyboardController k(argv[1], keys, names, powers, 2);

	const char * motorNames[8] = {
		(char*)"thigh1FL_a",
		(char*)"thigh2FL_a",
		(char*)"thigh1FR_a",
		(char*)"thigh2FR_a",
		(char*)"thigh1BL_a",
		(char*)"thigh2BL_a",
		(char*)"thigh1BR_a",
		(char*)"thigh2BR_a",
	};

	const char * jointNames[8] = {
		(char*)"thigh1FL_j",
		(char*)"thigh2FL_j",
		(char*)"thigh1FR_j",
		(char*)"thigh2FR_j",
		(char*)"thigh1BL_j",
		(char*)"thigh2BL_j",
		(char*)"thigh1BR_j",
		(char*)"thigh2BR_j",
	};

	const char * endeffectorNames[4] = {
		(char*)"endeffectorFL",
		(char*)"endeffectorFR",
		(char*)"endeffectorBL",
		(char*)"endeffectorBR",
	};

	PIDController c;
	MujocoSystem mjSys("MinitaurLeg.xml");
	mjSys.setGraphics(true);

	Mediator m(&c, &mjSys);
	m.run(-1);

	return 0;
}
