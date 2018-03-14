#include <stdio.h>
#include <SDK.h>
#include <Motor.h>

class Example : public Behavior {
	void begin() {

	}

	void update() {
		//control the robot via its joints
		C -> mode = RobotCommand_Mode_JOINT;

		//enable joint 0, and give it an open loop command
		joint[0].setOpenLoop(0.1);
	}

	void end() {
		
	}
};

int main(int argc, char* argv[]){
	//create the minitaur with default settings
	init(RobotParams_Type_MINITAUR, argc, argv);

	//configure joints
	#define NUM_MOTORS 6
	const float zeros[NUM_MOTORS] = {0, 0, 0, 0, 0, 0};
	const float directions[NUM_MOTORS] = {1, 1, 1, 1, 1, 1};
	P -> joints_count = S -> joints_count = C -> joints_count = NUM_MOTORS;

	//set zeros and directions
	for (int i = 0; i < P -> joints_count; i++) {
		P -> joints[i].zero = zeros[i];
		P -> joints[i].direction = directions[i];
	}

	P -> limbs_count = 0;

	safetyShutoffEnable(false);
	softStartEnable(false);


	//our example behavior
	Example Example;

	//remove any built in behaviors, add our own
	behaviors.clear();
	behaviors.push_back(&Example);
	behaviors.begin();

	//set the debug rate for logging purposes
	setDebugRate(100);

	return begin();
}

void debug(){
	printf("%u\t%f\n", S -> millis, joint[0].getPosition());
}