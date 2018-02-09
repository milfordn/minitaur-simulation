#include "Noise.h"

NoiseFilter::NoiseFilter(mjtNum freq, mjtNum mag, int vsize) {
	this->time = 0;
	this->period = 1. / freq;
	this->magnitude = mag;
	this->additiveNoiseFrom = Eigen::Matrix<mjtNum, Eigen::Dynamic, 1>(0, vsize);
}

void NoiseFilter::step(mjtNum interval) {
	time += interval;
	if (time > period) {
		incrementVectors();
		time -= period;
	}
}

void NoiseFilter::applyNoise(mjtNum * v) {
	Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> interpolated = additiveNoiseFrom + (additiveNoiseTo - additiveNoiseFrom) * (time / period);
	for (int i = 0; i < interpolated.size(); i++) {
		v[i] += interpolated[i];
	}
}

void NoiseFilter::incrementVectors() {
	additiveNoiseFrom = additiveNoiseTo;
	for (int i = 0; i < additiveNoiseTo.size(); i++) {
		additiveNoiseTo[i] = magnitude * (1 - rand() / rand.max());
	}
}