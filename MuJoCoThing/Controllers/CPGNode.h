#ifndef CPG_H
#define CPG_H

#include "ModelController.h"

class CPGNode {
public:
	//A and B control the speed of convergence for X and Y respectively
	//mu controls frequency and/or amplitude?
	CPGNode(double a, double b, double mu);
	void step(double dt);
	void step(double dt, double measuredX);
	double getValue();
	void setPose(double wstance, double wswing);
private:
	double a, b, mu;
	double x, y;
	double wstance, wswing;
};

#endif