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
};
