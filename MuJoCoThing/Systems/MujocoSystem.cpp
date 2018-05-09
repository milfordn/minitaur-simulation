#include "MujocoSystem.h"
#include "../System.h"
#include "../render.h"
#include <iostream>
using std::cout;
using std::endl;
MujocoSystem::MujocoSystem(mjData * d, mjModel * m)
{
	mj_copyModel(model, m);
	mj_copyData(data, model, d);
}

MujocoSystem::MujocoSystem(char * file)
{
	char error[1000];
	this->model = mj_loadXML(file, NULL, error, sizeof(error));
	if (!this->model) {
		mju_error_s("Couldn't load model: %s", error);
		printf("%s\n", error);
	}
	else {
		this->data = mj_makeData(model);
	}

	this->realTime = true;
	this->lastTime = 0;
	this->data = mj_makeData(model);
}

MujocoSystem::~MujocoSystem()
{
	mj_deleteModel(model);
	mj_deleteData(data);
	if (graphics) close();
}

void MujocoSystem::setRealTime(bool b)
{
	this->realTime = b;
}

void MujocoSystem::setGraphics(bool b)
{
	if (b)
		init(model, 1024, 768);
	else if (graphics)
		close();

	this->graphics = b;
}

double MujocoSystem::step()
{
	if(!model) return 0;

	//read actuator data
	for (int i = 0; i < model->nu; i++) {
		string name = mj_id2name(model, mjtObj::mjOBJ_ACTUATOR, i);
		data->ctrl[i] = (*actuatorRef)[name];
	}

	mj_step2(model, data);

	//render
	if (graphics && shouldRender()) {
		render(data);
		lastRender = data->time;
		lastRealTime = clock();
	}

	mj_step1(model, data);

	//write sensor data
	//possible bug - gyro and accel both output 3 numbers, not 1 - might mess up indexing in sensordata
	for (int i = 0; i < model->nsensor; i++) {
		string name = mj_id2name(model, mjtObj::mjOBJ_SENSOR, i);
		(*sensorRef)[name] = data->sensordata[i];
	}

	//timekeeping
	double dt = data->time - lastTime;
	lastTime = data->time;
	return dt;
}

bool MujocoSystem::shouldRender()
{
	if (realTime)
		return data->time - lastRender > 1.0 / 60.0;
	else
		return (clock() - lastRealTime) / (double)CLOCKS_PER_SEC > 1.0 / 60.0;
}
