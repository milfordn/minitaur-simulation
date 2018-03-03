#include "Noise.h"

std::default_random_engine Noise::rand;

Noise::Noise(mjtNum freq, mjtNum mag, Eigen::Index vsize) {
	this->elapsed = 0;
	this->period = 1. / freq;
	this->magnitude = Eigen::VectorXd::Constant(vsize, mag);
	this->additiveNoiseTo = Eigen::Matrix<mjtNum, Eigen::Dynamic, 1>(vsize);
	incrementVectors();
	incrementVectors();
}

Noise::Noise(mjtNum freq, Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> mag, Eigen::Index vsize) {
	this->elapsed = 0;
	this->period = 1. / freq;
	this->magnitude = mag;
	this->additiveNoiseTo = Eigen::Matrix<mjtNum, Eigen::Dynamic, 1>(vsize);
	incrementVectors();
	incrementVectors();
}

void Noise::step(mjtNum interval) {
	elapsed += interval;
	if (elapsed > period) {
		incrementVectors();
		elapsed -= period;
	}
}

void Noise::applyNoise(mjtNum * v) {
	Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> interpolated = additiveNoiseFrom + (additiveNoiseTo - additiveNoiseFrom) * (elapsed / period);
	for (int i = 0; i < interpolated.size(); i++) {
		v[i] += interpolated[i];
	}
}

Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> Noise::applyNoise(Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> v) {
	return v + additiveNoiseFrom + (additiveNoiseTo - additiveNoiseFrom) * (elapsed / period);
}

void Noise::incrementVectors() {
	additiveNoiseFrom = additiveNoiseTo;
	for (int i = 0; i < additiveNoiseTo.size(); i++) {
		additiveNoiseTo[i] = 2 * magnitude[i] * (0.5 - (double)rand() / rand.max());
	}
}