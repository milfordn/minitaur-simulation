#include "NoiseFilter.h"
#include "Eigen/Dense"

std::default_random_engine NoiseFilter::rand;

NoiseFilter::NoiseFilter(mjtNum freq, mjtNum mag, Eigen::Index vsize) {
	this->elapsed = 0;
	this->period = 1. / freq;
	this->magnitude = Eigen::VectorXd::Constant(vsize, mag);

	for (int i = 0; i < vsize; i++) {
		this->magnitude[i] = mag;
	}

	this->additiveNoiseTo = Eigen::Matrix<mjtNum, Eigen::Dynamic, 1>(vsize);
	incrementVectors();
	incrementVectors();
}

NoiseFilter::NoiseFilter(mjtNum freq, Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> mag, Eigen::Index vsize) {
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

Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> NoiseFilter::applyNoise(Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> v) {
	return v + additiveNoiseFrom + (additiveNoiseTo - additiveNoiseFrom) * (elapsed / period);
}

void NoiseFilter::incrementVectors() {
	additiveNoiseFrom = additiveNoiseTo;
	for (int i = 0; i < additiveNoiseTo.size(); i++) {
		additiveNoiseTo[i] = 2 * magnitude[i] * (0.5 - (double)rand() / rand.max());
	}
}