#include "MinitaurControllerCPG.h"

MinitaurControllerCPG::MinitaurControllerCPG(char * f) : ModelController(f),
	Net(4, 1, 50, mjPI / 4)
{
	this->fl = new LegControllerCPG(model, data, "thigh1FL_spos", "thigh1FL_a", "thigh2FL_spos", "thigh2FL_a", Net.getNode(0));
	this->fr = new LegControllerCPG(model, data, "thigh1FR_spos", "thigh1FR_a", "thigh2FR_spos", "thigh2FR_a", Net.getNode(1));
	this->bl = new LegControllerCPG(model, data, "thigh1BL_spos", "thigh1BL_a", "thigh2BL_spos", "thigh2BL_a", Net.getNode(2));
	this->br = new LegControllerCPG(model, data, "thigh1BR_spos", "thigh1BR_a", "thigh2BR_spos", "thigh2BR_a", Net.getNode(3));

	Eigen::MatrixXd coupling;
	coupling.resize(4, 4);
	coupling << 0, -1, -1, 1,
				-1, 0, 1, -1,
				-1, 1, 0, -1,
				1, -1, -1, 0;

	Net.setCoupling(coupling);
}

void MinitaurControllerCPG::step() {
	mjtNum dt = data->time - lastTime;
	lastTime = data->time;
	Net.applyCoupling();
	Net.step(dt);
	
	fl->step();
	fr->step();
	bl->step();
	br->step();

	/*for (int i = 0; i < 4; i++)
		printf("%f | ", Net.getNode(i)->getValueX());
	putchar('\n');*/
}

