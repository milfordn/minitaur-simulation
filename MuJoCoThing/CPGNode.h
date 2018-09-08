#ifndef CPG_H
#define CPG_H

class CPGNode {
public:
	//A and B control the speed of convergence for X and Y respectively
	//mu controls frequency and/or amplitude
	CPGNode(double alpha, double beta, double range, double swing, double stance, double x, double y);
	void step(double dt);
	double getAngle();
	double getLength();
private:
	double alpha, beta, b, mu;
	double x, y;
	double wstance, wswing;
};

#endif
