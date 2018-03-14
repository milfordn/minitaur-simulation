#ifndef CPGMINI_H
#define CPGMINI_H

#include <vector>
#include "LegControllerCPG.h"
#include "../CPGNetwork.h"

using std::vector;

class MinitaurControllerCPG : public ModelController {
public:
	MinitaurControllerCPG(char * f);
	void step() override;
private:
	LegControllerCPG *fl, *fr, *bl, *br;
	CPGNetwork Net;
	mjtNum lastTime;
};

#endif