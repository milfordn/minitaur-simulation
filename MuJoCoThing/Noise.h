#ifndef NOISE_H
#define NOISE_H

#include "Eigen/Core"
#include "include/mujoco.h"
#include <random>

class NoiseFilter {
public:
	NoiseFilter(mjtNum frequency, mjtNum magnitude, Eigen::Index vectorSize);
	void applyNoise(mjtNum *);
	void step(mjtNum interval);
private:
	void incrementVectors();
	Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> additiveNoiseFrom;
	Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> additiveNoiseTo;	
	mjtNum elapsed;
	mjtNum period;
	mjtNum magnitude;
	static std::default_random_engine rand;
};

#endif