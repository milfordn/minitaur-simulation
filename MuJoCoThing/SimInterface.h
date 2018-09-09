#ifndef SIMINT_H
#define SIMINT_H

#include "Systems/MujocoSystem.h"
#include "Mediator.h"

extern "C" {

	MujocoSystem * sim_init(char * file) {
		return new MujocoSystem(file);
	}
	MujocoSystem * sim_init(mjModel * m, mjData * d) {
		return new MujocoSystem(m, d);
	}

	Mediator * med_init(Controller * c, System * s) {
		return new Mediator(c, s);
	}
	void med_run(Mediator * m, double time) {
		m->run(time);
	}
}

#endif