#include "ModelController.h"
#include "LegPositionController.h"

class ForwardVelocityController : public ModelController {
public :
	ForwardVelocityController(const char *, const char * [], const char * [], const char * []);
	~ForwardVelocityController();
	void step() override;
private:
	LegPositionController *frontLeft;
	LegPositionController *frontRight;
	LegPositionController *backLeft;
	LegPositionController *backRight;
	double pair1Theta;
	double pair2Theta;
	double pair1Speed;
	double pair2Speed;
	double time[1000];
	double length[1000];
	double angle[1000];
	double t;
	unsigned long tick;
};
