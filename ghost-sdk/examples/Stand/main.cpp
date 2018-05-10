/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Tom Jacobs <tom.jacobs@ghostrobotics.io> and Avik De <avik@ghostrobotics.io>
 */
#include <stdio.h>
#include <SDK.h>
#include <Motor.h>

#if defined(ROBOT_MINITAUR)
// Subject to change for individual robots
//const float motZeros[8] = {5.60, 5.65, 4.80, 2.92, 4.47, 1.28, 5.50, 2.98};
//motors that need to go up:1, 3, 5, 7
//motors that need to go down: 0, 2, 4, 6
 
const float motZeros[8] = {5.45, 5.80, 4.65, 3.07, 4.32, 1.43, 5.35, 3.13};
//const float motZeros[8] = {5.80, 5.45, 5.00, 2.72, 4.67, 1.08, 5.70, 2.78};
//this direction const is for the current config of motors
const float motDirections[8] = {1, 1, 1, 1, -1, -1, -1, -1};
#endif

/**
 * See "Getting started" in the documentation for a walk through guide.
 * http://ghostrobotics.gitlab.io/SDK/
 */
class Stand: public Behavior {
public:

	// ::begin() is called once when the behavior starts
	void begin() {

		// Command limbs
		C->mode = RobotCommand_Mode_LIMB;
	}

	// ::update() is called once per loop while the behavior is running
	void update() {

		// Calculate leg angle and extension values:
		// C->behavior.pose.position.z can be commanded from the joystick with the left vertical axis.
		// We map this using map() to the desired leg extension, so that the joystick can be used to raise
		// and lower the standing height between 0.14 and 0.25 m.
		float extension = map(C->behavior.pose.position.z, -1.0, 1.0, 0.14, 0.25);

		// And angle is calculated as the negative pitch value of the robot to keep the legs pointing down.
		float angle = S->imu.euler.x;

		// For each of the four legs:
		for (int i = 0; i < P->limbs_count; ++i)
		{
			// Set limb type
			P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;

			// Set leg angles
			limb[i].setGain(ANGLE, 0.8, 0.03);
			limb[i].setPosition(ANGLE, angle);

			// Set leg extensions
			limb[i].setGain(EXTENSION, 120, 4);
			limb[i].setPosition(EXTENSION, extension);
		}
	}

	// ::end() is called when the behavior is stopped
	void end() {

	}

	// ::signal() is called when the signal button on the remote controller is pressed
	void signal(uint32_t signal_code) {
		printf("Got signal %ld\r\n", signal_code);
	}
};

// Main
int main(int argc, char *argv[]) {
#if defined(ROBOT_MINITAUR)
	// Create MINITAUR
	init(RobotParams_Type_MINITAUR, argc, argv);

	// Set motor zeros
	for (int i = 0; i < P->joints_count; ++i){
		P -> joints[i].zero = motZeros[i];
		P -> joints[i].direction = motDirections[i];
	}
#elif defined(ROBOT_MINITAUR_E)

	// Create MINITAUR_E
	init(RobotParams_Type_MINITAUR_E, argc, argv);

	// Set joystick type
	JoyType joyType = JoyType_FRSKY_XSR;
	ioctl(JOYSTICK_FILENO, IOCTL_CMD_JOYSTICK_SET_TYPE, &joyType);
#else
#error "Define robot type in preprocessor"
#endif

	// Create, add, and start our behavior
	Stand stand;
	behaviors.clear();
	behaviors.push_back(&stand);
	stand.begin();

	// Run
	return begin();
}

// Debug is called at the DEBUG_RATE
void debug() {
	printf("IMU Y: %lf\r\n", S->imu.euler.x);
}
