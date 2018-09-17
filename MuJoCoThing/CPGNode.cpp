#include "CPGNode.h"
#include <cmath>
#include <iostream>

using std::cout;
using std::endl;

CPGNode::CPGNode(double alpha, double beta, double range, double swing, double stance, double x, double y) {
	this->alpha = alpha;
	this->beta = beta;
	this->mu = range * range;
	this->x = x;
	this->y = y;
	this->wswing = swing;
	this->wstance = stance;
	this->coupling = 0;
}

void CPGNode::step(double dt) {
	double r = x * x + y * y;
	double w = wswing / (exp(-y) + 1) + wstance / (exp(y) + 1);
	double dx = alpha * (mu - r) * x - w * y;
	double dy = beta * (mu - r) * y + w * x + coupling;
	x += dx * dt;
	y += dy * dt;
}

double CPGNode::getAngle() {
	return x;
}

double CPGNode::getLength() {
	return y;
}

void CPGNode::setCoupling(double coupling)
{
	this->coupling = coupling;
}
