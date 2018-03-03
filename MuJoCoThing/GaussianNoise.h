#ifndef GAUSNOISE_H
#define GAUSNOISE_H

#include "Eigen/Core"
#include "include/mujoco.h"
#include <vector>
#include <random>

using std::vector;

class GaussianNoise {
public:
	GaussianNoise(mjtNum mean, mjtNum stdev, int vectorSize);
	GaussianNoise(Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> means, Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> stdevs);
	void applyNoise(mjtNum *);
	Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> applyNoise(Eigen::Matrix<mjtNum, Eigen::Dynamic, 1>);
	void step();
private:
	Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> additiveNoise;
	Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> means;
	Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> stdevs;
	static std::normal_distribution<mjtNum> distribution;
	static std::default_random_engine rand;
};

#endif