#include "CPGNetwork.h"

using std::vector;

CPGNetwork::CPGNetwork(Eigen::Index size, double a, double b, double amplitude) {
	for (int i = 0; i < size; i++) {
		Nodes.push_back(CPGNode(a, b, amplitude));
	}
	coupling = Eigen::MatrixXd::Zero(size, size);
}

Eigen::VectorXd CPGNetwork::getYVector() {
	Eigen::VectorXd toReturn = Eigen::VectorXd::Zero(Nodes.size());
	for (int i = 0; i < Nodes.size(); i++) {
		toReturn[i] = Nodes[i].getValueY();
	}
	return toReturn;
}

Eigen::VectorXd CPGNetwork::getXVector() {
	Eigen::VectorXd toReturn = Eigen::VectorXd::Zero(Nodes.size());
	for (int i = 0; i < Nodes.size(); i++) {
		toReturn[i] = Nodes[i].getValueX();
	}
	return toReturn;
}

void CPGNetwork::step(mjtNum dt) {
	for (int i = 0; i < Nodes.size(); i++) {
		Nodes[i].step(dt);
	}
}

void CPGNetwork::applyCoupling() {
	Eigen::VectorXd couplingVector = coupling * this->getYVector();
	for (int i = 0; i < Nodes.size(); i++) {
		Nodes[i].setCoupling(couplingVector[i]);
	}
}

CPGNode * CPGNetwork::getNode(int i)
{
	return &(Nodes[i]);
}

void CPGNetwork::setCoupling(Eigen::MatrixXd m) {
	this->coupling = m;
}