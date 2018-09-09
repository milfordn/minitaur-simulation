#include <fstream>
#include "ModelController.h"
#include "LegPositionController.h"
#include "CPGNode.h"
#include "../pid.h"

class SingleUnitController : public ModelController {
public :
	SingleUnitController(const char *, const char * [], const char * [], const char * []);
	~SingleUnitController();
	void keyboardCallback(GLFWwindow*, int key, int, int act, int mod);
	void step() override;
private:
	LegPositionController *frontLeft;
	LegPositionController *frontRight;
	LegPositionController *backLeft;
	LegPositionController *backRight;

	CPGNode *pattern1;
	CPGNode *pattern2;
	CPGNode *pattern3;
	CPGNode *pattern4;

	double t;

	double PI = 3.14159265359;
	double e = 2.718281828459;
	int datapoints = 20;
	std::ofstream output;
	unsigned long tick = 0;
};
