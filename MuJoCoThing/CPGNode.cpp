#include "CPGNode.h"
#include <cmath>

CPGNode::CPGNode(double a, double b, double range) {
	this->a = a;
	this->b = b;
	this->mu = range * range;
	this->x = 0;
	this->y = 0;
}

void CPGNode::step(double dt) {
	double r = x * x + y * y;
	double w = wstance / (exp(-b * y) + 1) + wswing / (exp(b * y) + 1);
	double dx = a * (mu - r) * x - w * y;
	double dy = b * (mu - r) * y + w * x;

	x += dx * dt;
	y += dy * dt;
}

void CPGNode::step(double dt, double measuredX) {
	this->x = measuredX;
	this->step(dt);
}

double CPGNode::getValue() {
	return x;
}

void CPGNode::setPose(double wstance, double wswing) {
	this->wstance = wstance;
	this->wswing = wswing;
}