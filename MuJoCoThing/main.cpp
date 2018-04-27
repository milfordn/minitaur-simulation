#include "./Mediator.h"
#include "./Systems/MujocoSystem.h"
#include "Controller.h"
#include "./NewControllers/PIDController.h"


int main(int argc, char ** argv) {
	mj_activate("mjkey.txt");

	PIDController c;
	MujocoSystem mjSys("MinitaurLeg.xml");
	mjSys.setGraphics(true);

	Mediator m(&c, &mjSys);
	m.run(-1);

	return 0;
}
