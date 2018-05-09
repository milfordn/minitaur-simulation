#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <string>
#include <unordered_map>
#include <vector>
using std::unordered_map;
using std::string;

class Controller {
public:
	virtual void step(double dt) {};
	void setSensorRef(unordered_map<string, double> *);
	void setActuatorRef(unordered_map<string, double> *);
protected:
	unordered_map<string, std::vector<double>> * sensorRef;
	unordered_map<string, double> * actuatorRef;
};

#endif
