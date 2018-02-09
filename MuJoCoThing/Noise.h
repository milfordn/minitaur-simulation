#ifndef NOISE_H
#define NOISE_H

#include "Eigen/Core"
#include "include/mujoco.h"
#include <random>

class NoiseFilter {
public:
	NoiseFilter(mjtNum, mjtNum, int);
	void applyNoise(mjtNum *);
	void step(mjtNum);
private:
	void incrementVectors();
	Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> additiveNoiseFrom;
	Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> additiveNoiseTo;	
	mjtNum time;
	mjtNum period;
	mjtNum magnitude;
	static std::default_random_engine rand;
};

#endif