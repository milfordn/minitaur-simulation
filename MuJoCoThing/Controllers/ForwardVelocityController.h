#include "ModelController.h"
#include "LegPositionController.h"
#include "../pid.h"
#include <fstream>

using namespace std;
class ForwardVelocityController : public ModelController {
public :
	ForwardVelocityController(const char *, const char * [], const char * [], const char * []);
	~ForwardVelocityController();
	double calculateTheta(double speed, double t, double offset, double bias);
	double calculateLength(double speed, double t, double offset, double height);
	double calculateStiffness(double speed, double t, double offset);
	void keyboardCallback(GLFWwindow*, int key, int, int act, int mod);
	void step() override;
private:
	LegPositionController *frontLeft;
	LegPositionController *frontRight;
	LegPositionController *backLeft;
	LegPositionController *backRight;
	pid *p; //High-level PID controller for controlling pitch
	pid *r; //High-level PID controller for controlling roll
	double pair1Theta;
	double pair2Theta;
	double speed;
	double pair1Speed;
	double pair2Speed;
	double time[1000];
	double length[1000];
	double angle[1000];
	double t;
	unsigned long tick;

	double averageBias = 0;
	double averageStoop = 0;
	double PI = 3.14159265359;
	double e = 2.718281828459;
	int datapoints = 20;
	ofstream output;

};
