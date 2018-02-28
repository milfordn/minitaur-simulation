#include "ModelController.h"
#include "LegPositionController.h"

class PositionController : public ModelController {
public :
	PositionController(const char *, const char * [], const char * [], const char * []);
	~PositionController();
	void step() override;
private:
	LegPositionController *frontLeft;
	LegPositionController *frontRight;
	LegPositionController *backLeft;
	LegPositionController *backRight;
	int bodyID;
	unsigned long tick;
};

