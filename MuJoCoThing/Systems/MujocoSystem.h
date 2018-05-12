#ifndef MJSYSTEM_H
#define MJSYSTEM_H

#include "../include/mujoco.h"
#include "../System.h"
#include "../render.h"
#include <ctime>

class MujocoSystem : public System {
public:
	MujocoSystem(mjData * d, mjModel * m);
	MujocoSystem(char * file);
	~MujocoSystem();
	void setGraphics(bool b);
	void setRealTime(bool b);
	double step() override;
private:
	bool shouldRender();
	mjData * data;
	mjModel * model;
	mjRender * render;
	bool realTime;
	double lastTime, lastRender;
	clock_t lastRealTime;
};

#endif