#include "CustomSimulate.h"
#include "KeyboardController.h"
#include "PIDController.h"
#include "PositionController.h"
#include "include/glfw3.h"
#include "ModelController.h"
#include "NoiseFilter.h"
#include <iostream>

int main(int argc, char ** argv) {
	mj_activate("mjkey.txt");
	//ModelController m("MinitaurLeg.xml");

	char * names[2] = { (char*)"motor_a", (char*)"motor_c" };
	int keys[2] = { GLFW_KEY_A, GLFW_KEY_D };
	double powers[2] = { -0.01, 0.01 };
	
	KeyboardController k(argv[1], keys, names, powers, 2);
	
	PositionController p(argv[1], names, 2);
		
	run(&p);

	//testing NoiseFilter
	/*NoiseFilter f(0.1, 1, 4);
	Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> base((Eigen::Index)4);
	base.setZero();

	for (int i = 0; i < 100; i++) {
		f.step(1);
		Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> res = f.applyNoise(base);
		printf("%f, %f, %f, %f\n", res[0], res[1], res[2], res[3]);
	}
	scanf_s("");*/


	return 0;
}