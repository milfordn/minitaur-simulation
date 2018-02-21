#include "ModelController.h"
#include "pid.h"

class PositionController : public ModelController {
public :
	PositionController(const char *, char * [], int size, char *);
	~PositionController();
	void step() override;
private:
	unsigned long tick;
	int * actuatorIDs;
	int endeffectorID;
	int size;
	pid *m1;
	pid *m2;
	pid *angleController;
	pid *lengthController;
	double setposY = 1;
	double setposX = 0.5;
	
};

