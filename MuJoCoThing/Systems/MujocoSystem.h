#ifndef MJSYSTEM_H
#define MJSYSTEM_H

#include "../include/mujoco.h"
#include "../System.h"
#include <ctime>

class MujocoSystem : public System {
public:
	MujocoSystem(mjModel * m, mjData * d);
	MujocoSystem(char * file);
	~MujocoSystem();
	void setGraphics(bool b);
	void setRealTime(bool b);
	void reset();
	double step() override;
private:
	bool shouldRender();
	mjData * data;
	mjModel * model;
	bool graphics, realTime;
	double lastTime, lastRender;
	clock_t lastRealTime;
};

#endif
