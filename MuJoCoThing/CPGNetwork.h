#ifndef CPGNET_H
#define CPGNET_H

#include <vector>
#include "Eigen/Core"
#include "Controllers/CPGNode.h"

using std::vector;
using Eigen::Matrix;

class CPGNetwork {
public:
	CPGNetwork(Eigen::Index size, double a, double b, double amplitude);
	void step(mjtNum dt);
	Matrix<double, Eigen::Dynamic, 1> getYVector();
	Matrix<double, Eigen::Dynamic, 1> getXVector();
	void setCoupling(Eigen::MatrixXd);
	void applyCoupling();
	CPGNode * getNode(int i);
private:
	vector<CPGNode> Nodes;
	Eigen::MatrixXd coupling;
};

#endif // !CPGNET_H
