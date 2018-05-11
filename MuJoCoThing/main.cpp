#include "Mediator.h"
#include "Systems/MujocoSystem.h"
#include "Controller.h"
#include "./NewControllers/LegControllerCPG.h"

int main(int argc, char ** argv) {
	mj_activate("mjkey.txt");

	LegControllerCPG c("thigh1_spos", "thigh2_spos", "thigh1_a", "thigh2_a");
	MujocoSystem mjSys("MinitaurLeg.xml");
	mjSys.setRealTime(true);
	mjSys.setGraphics(true);

	Mediator m(&c, &mjSys);
	m.run(-1);

	return 0;
}
