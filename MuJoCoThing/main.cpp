#include "Mediator.h"
#include "Controller.h"
#include "./Systems/MujocoSystem.h"
#include "./NewControllers/CPGController.h"

int main(int argc, char ** argv) {
	mj_activate("mjkey.txt");
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
		(char*)"foot1",
		(char*)"foot2",
		(char*)"foot3",
		(char*)"foot4",
	};
	double alpha = 1;
	double beta = 50;
	double range = 0.12;
	double stance = 10;
	double swing = 5;

	double params[28] = {alpha, beta, range, swing, stance, 1, 1, alpha, beta, range, swing, stance, 0, 1, alpha, beta, range, swing, stance, 1, 0, alpha, beta, range, swing, stance, 1, 1};

	CPGController c(params);
	MujocoSystem mjSys((char*)"MinitaurFull.xml");
	mjSys.setRealTime(true);
	mjSys.setGraphics(true);

	Mediator m(&c, &mjSys);
	m.run(50);

	return 0;
}
