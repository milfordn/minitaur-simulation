#ifndef CPG_H
#define CPG_H

#include "ModelController.h"

#define FEEDBACK_FAST 1
#define FEEDBACK_NONE 0
#define FEEDBACK_STOP -1

class CPGNode {
public:
	//A and B control the speed of convergence for X and Y respectively
	//mu controls amplitude
	CPGNode(double a, double b, double mu);
	void step(double dt);
	void setInitialConditions(double x, double y);

	double getValueX();
	double getValueY();

	//These are both measures of frequency
	void setPose(double wstance, double wswing);

	//1 for fast transition, 0 for no change, -1 for stop
	void setFeedbackType(int type);
	void setCoupling(double c);

	void setFeedback(double sensor);
private:
	double a, b, mu;
	double x, y;
	double wstance, wswing;
	double feedback, coupling;
	int feedbackType;
};

#endif