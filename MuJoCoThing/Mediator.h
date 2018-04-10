#ifndef MEDIATOR_H
#define MEDIATOR_H

#include "Controller.h"
#include "System.h"

using std::unordered_map;

class Mediator {
public:
	Mediator(Controller *, System *);
	void run(double ms);
private:
	double time;
	Controller * c;
	System * s;
	unordered_map<string, double> sensorData, actuatorData;
};

#endif