#include "CustomSimulate.h"
#include "Controllers/KeyboardController.h"
#include "Controllers/PIDController.h"
#include "Controllers/ModelController.h"
#include "Controllers/LegControllerSimple.h"
#include "GaussianNoise.h"

int main(int argc, char ** argv) {
	mj_activate("mjkey.txt");
	//ModelController m("MinitaurLeg.xml");

	char * names[2] = { (char*)"thigh1_a", (char*)"thigh2_a" };
	int keys[2] = { GLFW_KEY_A, GLFW_KEY_D };
	double powers[2] = { -0.01, 0.01 };
	
	//KeyboardController k(argv[1], keys, names, powers, 2);	
	//PIDController p(argv[1], names, 2);
	LegControllerSimple m("MinitaurLeg.xml");
	
	run(&m);

	//testing NoiseFilter
	//GaussianNoise g(Eigen::Vector4d(-5, -2, 2, 5), Eigen::Vector4d(2, 1, 2, 4));
	//Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> base(4);
	//base.setZero();

	//for (int i = 0; i < 100; i++) {
	//	g.step();
	//	Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> res = g.applyNoise(base);
	//	printf("%f, %f, %f, %f\n", res[0], res[1], res[2], res[3]);
	//}
	//scanf_s("");


	return 0;
}