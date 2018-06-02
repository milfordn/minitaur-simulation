#include "Mediator.h"
#include <cstdio>
#include <iostream>

Mediator::Mediator(Controller * nc, System * ns)
{
	this->c = nc;
	this->s = ns;

	c->setActuatorRef(&actuatorData);
	c->setSensorRef(&sensorData);
	s->setActuatorRef(&actuatorData);
	s->setSensorRef(&sensorData);

	time = 0;
}

//pass a negative number for infinite time
void Mediator::run(double secs)
{
	while (secs < 0 || this->time < secs) {
		double dt = s->step();
		c->step(dt);
		this->time += dt;
	}
	c->exit();
}
