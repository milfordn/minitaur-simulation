/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io> and Tom Jacobs <tom.jacobs@ghostrobotics.io>
 */
#include <stdio.h>
#include <SDK.h>
#include <Motor.h>

class Test : public Behavior
{
	void begin()
	{

	}
	void update()
	{
		// Control robot by joints
		C->mode = RobotCommand_Mode_JOINT;

		// Motor plugged in to port 0
		joint[0].setOpenLoop(0.1);         
	}

	void end()
	{

	}
};

int main(int argc, char *argv[])
{
	// Loads some default settings including motor parameters
	init(RobotParams_Type_MINITAUR, argc, argv);

	// Configure joints
	#define NUM_MOTORS 6
	const float zeros[NUM_MOTORS] = {0, 0, 0, 0, 0, 0};
	const float directions[NUM_MOTORS] = {1, 1, 1, 1, 1, 1};
	P->joints_count = S->joints_count = C->joints_count = NUM_MOTORS;
	for (int i = 0; i < P->joints_count; i++)
	{
		// Set zeros and directions
		P->joints[i].zero = zeros[i];
		P->joints[i].direction = directions[i];
	}

	// No limbs, only 1DOF joints
	P->limbs_count = 0; 

	// Disable motor safety shutdown and soft start
	safetyShutoffEnable(false);
	softStartEnable(false);

	// Remove default behaviors from behaviors vector, create, add, and start ours
	Test test;
	behaviors.clear();
	behaviors.push_back(&test);
	behaviors.begin();

	return begin();
}
