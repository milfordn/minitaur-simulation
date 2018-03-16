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
	double pair1Length;
	double pair2Length;
	int pair1Incrementor;
	int pair2Incrementor;
};
