#include "Mediator.h"
#include "Systems/MujocoSystem.h"
#include "Controller.h"
#include "./NewControllers/PIDController.h"
/*
#include "CustomSimulate.h"
#include "./Controllers/KeyboardController.h"
#include "./Controllers/PIDController.h"
#include "./Controllers/PositionController.h"
#include "./Controllers/ModelController.h"
#include "./Controllers/LegControllerSimple.h"
#include "./Controllers/LegControllerCPG.h"
#include "./Controllers/ForwardVelocityController.h"
#include "./Controllers/SingleUnitController.h"
#include "GaussianNoise.h"
*/

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

	PIDController c;
	MujocoSystem mjSys("MinitaurLeg.xml");
	mjSys.setRealTime(false);
	mjSys.setGraphics(true);

	Mediator m(&c, &mjSys);
	m.run(5);

	return 0;
}
