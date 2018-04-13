#include "Mediator.h"

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
void Mediator::run(double ms)
{
	while (ms < 0 || this->time < ms) {
		double dt = s->step();
		c->step(dt);
		this->time += dt;
	}
}
