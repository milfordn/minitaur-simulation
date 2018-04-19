#ifndef MJSYSTEM_H
#define MJSYSTEM_H

#include "../include/mujoco.h"
#include "../System.h"

class MujocoSystem : public System {
public:
	MujocoSystem(mjData * d, mjModel * m);
	MujocoSystem(char * file);
	~MujocoSystem();
	void setGraphics(bool b);
	void setRealTime(bool b); //unimplemented
	double step() override;
private:
	mjData * data;
	mjModel * model;
	bool graphics, realTime;
	double lastTime;
};

#endif