#include "MinitaurControllerCPG.h"

const vector<vector<double>> NET_PARAMS = {
	{ 5, 50, 1, 3, 12, 0, 0 },
	{ 5, 50, 1, 3, 12, 0, 0 },
	{ 5, 50, 1, 3, 12, 0, 0 },
	{ 5, 50, 1, 3, 12, 0, 0 },
};

MinitaurControllerCPG::MinitaurControllerCPG() : Net(4, NET_PARAMS)
{
	this->fl = new LegControllerCPG("thigh1FL_spos", "thigh1FL_a", "thigh2FL_spos", "thigh2FL_a", Net.getNode(0));
	this->fr = new LegControllerCPG("thigh1FR_spos", "thigh1FR_a", "thigh2FR_spos", "thigh2FR_a", Net.getNode(1));
	this->bl = new LegControllerCPG("thigh1BL_spos", "thigh1BL_a", "thigh2BL_spos", "thigh2BL_a", Net.getNode(2));
	this->br = new LegControllerCPG("thigh1BR_spos", "thigh1BR_a", "thigh2BR_spos", "thigh2BR_a", Net.getNode(3));

	Eigen::MatrixXd coupling;
	coupling.resize(4, 4);
	coupling << 0, -1, -1, 1,
				-1, 0, 1, -1,
				-1, 1, 0, -1,
				1, -1, -1, 0;

	Net.setCoupling(coupling);

	for (int i = 0; i < 1000; i++)
		Net.step(0.05);
}

void MinitaurControllerCPG::step(double dt) {
	Net.applyCoupling();
	
	fl->step(dt);
	fr->step(dt);
	bl->step(dt);
	br->step(dt);
}

