#include "CPGNode.h"
#include <cmath>
#include <cstdio>

CPGNode::CPGNode(double a, double b, double range) {
	this->a = a;
	this->b = b;
	this->mu = range * range;
	this->x = 0;
	this->y = 0;
	this->feedbackType = FEEDBACK_NONE;
}

void CPGNode::step(double dt) {
	double r = x * x + y * y;
	double w = wstance / (exp(-b * y) + 1) + wswing / (exp(b * y) + 1);
	
	//sensor hit in the latter half of the swing phase during normal operation
	if (y < 0 && x > 0 && feedback > 0 && feedbackType == FEEDBACK_NONE) {
		this->feedbackType = FEEDBACK_FAST;
		puts("FAST TRANSITION");
	}
	//sensor not hit in the latter half of the swing phase during normal operation
	else if (y < 0 && x > 0 && feedback < 0 && feedbackType == FEEDBACK_NONE) {
		this->feedbackType = FEEDBACK_STOP;
		puts("STOP TRANSITION");
	}
	//we're in a specific feedback condition and the sensor changes
	else if (
		(feedback > 0 && feedbackType == FEEDBACK_STOP) ||
		(feedback < 0 && feedbackType == FEEDBACK_FAST)
		) {
		this->feedbackType = FEEDBACK_NONE;
		puts("NO FEEDBACK");
	}

	double u = 0;
	if (feedbackType == FEEDBACK_FAST) {
		u = y < 0 ? 1 : -1;
		u *= 300;
	}
	else if (feedbackType == FEEDBACK_STOP) {
		u = -w * x - coupling;
	}

	double dx = a * (mu - r) * x - w * y;
	double dy = b * (mu - r) * y + w * x + coupling;// +u;

	x += dx * dt;
	y += dy * dt;
}

void CPGNode::step(double dt, double measuredX) {
	this->x = measuredX;
	this->step(dt);
}

double CPGNode::getValueX() {
	return x;
}

double CPGNode::getValueY() {
	return y;
}

void CPGNode::setPose(double wstance, double wswing) {
	this->wstance = wstance;
	this->wswing = wswing;
}

void CPGNode::setFeedbackType(int type) {
	this->feedbackType = type;
}

void CPGNode::setCoupling(double c)
{
	this->coupling = c;
}

void CPGNode::setFeedback(double sensor) {
	this->feedback = sensor;
}