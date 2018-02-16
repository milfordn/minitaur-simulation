#ifndef NOISEF_H
#define NOISEF_H

#include "Eigen/Core"
#include "include/mujoco.h"
#include <random>

class NoiseFilter {
public:
	NoiseFilter(mjtNum frequency, mjtNum magnitude, Eigen::Index vectorSize);
	NoiseFilter(mjtNum frequency, Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> magnitude, Eigen::Index vectorSize);
	void applyNoise(mjtNum *);
	Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> applyNoise(Eigen::Matrix<mjtNum, Eigen::Dynamic, 1>);
	void step(mjtNum interval);
private:
	void incrementVectors();
	Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> additiveNoiseFrom;
	Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> additiveNoiseTo;	
	mjtNum elapsed;
	mjtNum period;
	Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> magnitude;
	static std::default_random_engine rand;
};

#endif