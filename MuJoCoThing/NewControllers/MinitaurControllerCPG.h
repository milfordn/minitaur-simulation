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
	void setSensorRef(unordered_map<string, vector<double>> *) override;
	void setActuatorRef(unordered_map<string, double> *) override;
private:
	LegControllerCPG *fl, *fr, *bl, *br;
	CPGNetwork Net;
};

#endif