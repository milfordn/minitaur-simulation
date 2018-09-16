#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <string>
#include <unordered_map>
#include <vector>
using std::unordered_map;
using std::string;
using std::vector;

class Controller {
public:
	virtual void step(double dt) {};
	virtual double exit() { return 0; };
	virtual void setSensorRef(unordered_map<string, std::vector<double>> *);
	virtual void setActuatorRef(unordered_map<string, double> *);
protected:
	unordered_map<string, std::vector<double>> * sensorRef;
	unordered_map<string, double> * actuatorRef;
};

#endif
