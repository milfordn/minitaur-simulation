#include "Noise.h"
#include <ctime>

std::default_random_engine NoiseFilter::rand;

NoiseFilter::NoiseFilter(mjtNum freq, mjtNum mag, Eigen::Index vsize) {
	this->elapsed = 0;
	this->period = 1. / freq;
	this->magnitude = mag;
	this->additiveNoiseTo = Eigen::Matrix<mjtNum, Eigen::Dynamic, 1>(vsize);
	incrementVectors();
	incrementVectors();
}

void NoiseFilter::step(mjtNum interval) {
	elapsed += interval;
	if (elapsed > period) {
		incrementVectors();
		elapsed -= period;
	}
}

void NoiseFilter::applyNoise(mjtNum * v) {
	Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> interpolated = additiveNoiseFrom + (additiveNoiseTo - additiveNoiseFrom) * (elapsed / period);
	for (int i = 0; i < interpolated.size(); i++) {
		v[i] += interpolated[i];
	}
}

void NoiseFilter::incrementVectors() {
	additiveNoiseFrom = additiveNoiseTo;
	for (int i = 0; i < additiveNoiseTo.size(); i++) {
		additiveNoiseTo[i] = 2 * magnitude * (0.5 - (double)rand() / rand.max());
	}
}