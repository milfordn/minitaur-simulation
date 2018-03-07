#include <SDK.h>
#include <main.h>

class example_behavior;

int main(int arc, char* argv[]){
	//create the minitaur lol
	init(RobotParams_Type_MINITAUR, argc, argv);

	example_behavior example;

	//remove any built in behaviors, add our own
	behaviors.clear();
	behaviors.push_back(&example);
	example begin();

}