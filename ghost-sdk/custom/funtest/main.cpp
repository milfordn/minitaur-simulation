/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Gavin Kenneally, Avik De, Turner Topping <gavin@ghostrobotics.io> <avik@ghostrobotics.io> <turner@ghostrobotics.io>
 */
#include <stdio.h>
#include <SDK.h>
#include <math.h>
#include <Smath.h>
#include <Motor.h>
#include <Behavior.h>

class MyBehavior : public Behavior {
public:
	float posDes; //Desired position

	uint32_t tLast; // System time @ last state change
	
	void begin() {
		posDes = 0; // Initialize desired position to zero
		tLast = S->millis;// Set tLast at onset 
	}

	void update() {
		C->mode = RobotCommand_Mode_JOINT;

		// joint[0].getTorqueEst(); // N.m?
		// S->millis
		for (int i = 0; i < 2; i++)
		{
			joint[i].setGain(0.3, 0.002); //Sets P and D gains for position based control
		
			joint[i].setPosition(0);//Set motor position to newly incremented desired position.	
		}
	}

	bool running() {
		return true;
	}

	void end() {
	}
};

void debug() {
	for (int i = 0; i < 2; i++)
		printf("%f\t", joint[i].getPosition());
	printf("\n");
}

int main(int argc, char *argv[]) {
	init(RobotParams_Type_MINITAUR, argc, argv);
	setDebugRate(100);
	MyBehavior myBehavior; //Declare instance of our behavior
	//Disable the safety shut off feature:
	//IT IS POSSIBLE TO DAMAGE THE MOTOR; BE CAREFUL WHEN USING
	//BEHAVIORS WITHOUT THIS FAILSAFE 
	safetyShutoffEnable(false);
	//Disable the softStart feature
	softStartEnable(false);
	//Remove default behaviors from behaviors vector
	behaviors.clear();
	//add our behavior to behaviors vector
	behaviors.push_back(&myBehavior);

	return begin();
}

