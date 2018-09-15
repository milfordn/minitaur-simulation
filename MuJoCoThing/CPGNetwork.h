#ifndef CPGNET_H
#define CPGNET_H

#include <vector>
#include "Eigen/Core"
#include "CPGNode.h"

using std::vector;
using Eigen::Matrix;

class CPGNetwork {
public:
	CPGNetwork(Eigen::Index size, vector<vector<double>> params);
	void step(double dt);
	Matrix<double, Eigen::Dynamic, 1> getYVector();
	Matrix<double, Eigen::Dynamic, 1> getAngleVector();
	void setCoupling(Eigen::MatrixXd);
	void applyCoupling();
	CPGNode * getNode(int i);
private:
	vector<CPGNode> Nodes;
	Eigen::MatrixXd coupling;
};

#endif // !CPGNET_H
