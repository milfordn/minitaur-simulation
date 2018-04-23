#include <stdio.h>
#include <SDK.h>
#include <Motor.h>
#include <math.h>

class Example : public Behavior {
	void begin() {
		//control the robot via its joints
		C -> mode = RobotCommand_Mode_JOINT;
	}

	void update() {

		//enable joint 0, and give it an open loop command
		//joint[0].setOpenLoop(0.1);

		//set pd gain, proportional followed by derivative
		// joint[0].setGain(0.4, 0.00286);

		// for(int i = 0; i < 2000000; i++){
		// 	if (i == 500000)
		// 		joint[0].setPosition(-2);
		// 	if (i == 1000000)
		// 		joint[0].setPosition(-1);
		// 	if (i == 1500000)
		// 		joint[0].setPosition(0);
		// 	if (i == 2000000){
		// 		joint[0].setPosition(1);
		// 		i = 0;
		// 	}
		// }


		//values found for the pd controller
		//p critical = 2.4
		//t critical = 22 ms
		//p = 1.44
		//d = 0.00286

		//motor zeros (joint pointed vertically up)
		//j0 = 5.63
		//j1 = 5.50 
		//j2 = 2.96
		//j3 = 4.60
		//j4 = 2.98
		//j5 = 5.39
		//j6 = 4.36
		//j7 = 1.28
	}

	void end() {
		
	}
};

int main(int argc, char* argv[]){
	//create the minitaur with default settings
	init(RobotParams_Type_MINITAUR, argc, argv);

	//configure joints
	#define NUM_MOTORS 8
	const float zeros[NUM_MOTORS] = {0, 0, 0, 0, 0, 0, 0, 0};
	const float directions[NUM_MOTORS] = {1, 1, 1, 1, 1, 1, 1, 1};
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
	setDebugRate(10);

	return begin();
}

void debug(){
	for(int i = 0; i < 8; i++)
		printf("%f \t", joint[i].getRawPosition());
	printf("\n");
}