#include "ModelController.h"
#include "LegPositionController.h"

class RaibertPositionController : public ModelController {
public :
	RaibertPositionController(const char *, const char * [], const char * [], const char * []);
	~RaibertPositionController();
	void step() override;
private:
	LegPositionController *frontLeft;
	LegPositionController *frontRight;
	LegPositionController *backLeft;
	LegPositionController *backRight;
	int bodyID;
	unsigned long tick;
};

