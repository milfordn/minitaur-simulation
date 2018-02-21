#include "ModelController.h"
#include "pid.h"

class PositionController : public ModelController {
public :
	PositionController(const char *, char * [], int size);
	~PositionController();
	void step() override;
private:
	unsigned long tick;
	int * actuatorIDs;
	int size;
	pid *m1;
	pid *m2;
	int setposY;
	int setposX;
};

