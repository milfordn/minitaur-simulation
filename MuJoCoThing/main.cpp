#include "CustomSimulate.h"
#include "KeyboardController.h"
#include "include\glfw3.h"
#include "ModelController.h"
#include "NoiseFilter.h"

int main(int argc, char ** argv) {
	mj_activate("mjkey.txt");
	//ModelController m("MinitaurLeg.xml");
	char * names[8] = { 
		"thigh1_a", "thigh2_a", 
		"thigh1_a", "thigh2_a", 
		"thigh1_a", "thigh2_a",
		"thigh1_a", "thigh2_a"
	};

	int keys[8] = { 
		GLFW_KEY_A, GLFW_KEY_A, 
		GLFW_KEY_D, GLFW_KEY_D,
		GLFW_KEY_W, GLFW_KEY_W,
		GLFW_KEY_S, GLFW_KEY_S
	};

	double pow = 0.01;
	double powers[8] = { 
		-pow, -pow,
		pow, pow,
		-pow, pow,
		pow, -pow
	};

	KeyboardController k("MinitaurLeg.xml", keys, names, powers, 8);

	run(&k);

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