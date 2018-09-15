#ifndef CPGMINI_H
#define CPGMINI_H

#include <vector>
#include "LegControllerCPG.h"
#include "../CPGNetwork.h"

using std::vector;

class MinitaurControllerCPG : public Controller {
public:
	MinitaurControllerCPG();
	void step(double dt) override;
private:
	LegControllerCPG *fl, *fr, *bl, *br;
	CPGNetwork Net;
};

#endif