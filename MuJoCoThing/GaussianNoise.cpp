#include "GaussianNoise.h"

std::normal_distribution<mjtNum> GaussianNoise::distribution(1, 1);
std::default_random_engine GaussianNoise::rand;

GaussianNoise::GaussianNoise(mjtNum mean, mjtNum stdev, int vectorSize) {
	this->additiveNoise = Eigen::VectorXd(vectorSize);
	this->means = Eigen::VectorXd::Constant(vectorSize, mean);
	this->stdevs = Eigen::VectorXd::Constant(vectorSize, stdev);
}

GaussianNoise::GaussianNoise(Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> means, Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> stdevs) {
	if (means.size() != stdevs.size()) throw;
	this->additiveNoise = Eigen::VectorXd::Zero(means.size());
	this->means = means;
	this->stdevs = stdevs;
}

Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> GaussianNoise::applyNoise(Eigen::Matrix<mjtNum, Eigen::Dynamic, 1> vIn) {
	return vIn + additiveNoise;
}

void GaussianNoise::applyNoise(mjtNum * vIn) {
	for (int i = 0; i < additiveNoise.size(); i++) {
		vIn[i] += additiveNoise[i];
	}
}

void GaussianNoise::step() {
	for (int i = 0; i < additiveNoise.size(); i++) {
		additiveNoise[i] = distribution(rand);
	}

	additiveNoise = additiveNoise.cwiseProduct(stdevs) + means;
}