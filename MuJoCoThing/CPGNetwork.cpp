#include "CPGNetwork.h"

using std::vector;

CPGNetwork::CPGNetwork(Eigen::Index size, vector<vector<double>> params) {
	for (int i = 0; i < size; i++) {
		vector<double> nodeParams = params[i];
		Nodes.push_back(CPGNode(nodeParams[0], nodeParams[1], nodeParams[2], nodeParams[3], nodeParams[4], nodeParams[5], nodeParams[6]));
	}
	coupling = Eigen::MatrixXd::Zero(size, size);
}

Eigen::VectorXd CPGNetwork::getYVector() {
	Eigen::VectorXd toReturn = Eigen::VectorXd::Zero(Nodes.size());
	for (int i = 0; i < Nodes.size(); i++) {
		toReturn[i] = Nodes[i].getLength();
	}
	return toReturn;
}

Eigen::VectorXd CPGNetwork::getAngleVector() {
	Eigen::VectorXd toReturn = Eigen::VectorXd::Zero(Nodes.size());
	for (int i = 0; i < Nodes.size(); i++) {
		toReturn[i] = Nodes[i].getAngle();
	}
	return toReturn;
}

void CPGNetwork::step(double dt) {
	applyCoupling();
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