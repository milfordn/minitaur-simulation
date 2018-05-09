#ifndef CPGCTRL_H
#define CPGCTRL_H

#include "../Controller.h"
#include "../pid.h"
#include "../CPGNode.h"
class CPGController : public Controller {
public:
	CPGController(double p[28]);
	void step(double dt) override;
private:
	CPGNode* backLeft;
	CPGNode* backRight;
	CPGNode* frontLeft;
	CPGNode* frontRight;
	double time;
  pid* motors[8];

  char * motorNames[8] = {
    (char*)"thigh1FL_a",
    (char*)"thigh2FL_a",
    (char*)"thigh1FR_a",
    (char*)"thigh2FR_a",
    (char*)"thigh1BL_a",
    (char*)"thigh2BL_a",
    (char*)"thigh1BR_a",
    (char*)"thigh2BR_a",
  };
  char* sensorNames[8] = {
    (char*)"thigh1FL_spos",
    (char*)"thigh2FL_spos",
    (char*)"thigh1FR_spos",
    (char*)"thigh2FR_spos",
    (char*)"thigh1BL_spos",
    (char*)"thigh2BL_spos",
    (char*)"thigh1BR_spos",
    (char*)"thigh2BR_spos",
  };
};

#endif
