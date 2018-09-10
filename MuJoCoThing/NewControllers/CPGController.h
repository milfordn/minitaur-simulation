#ifndef CPGCTRL_H
#define CPGCTRL_H

#include <fstream>
#include "../Controller.h"
#include "../pid.h"
#include "../CPGNode.h"

using std::ofstream;

class CPGController : public Controller {
public:
	CPGController(double p[28]);
	void reset();
	void step(double dt) override;
	double exit() override;
private:
  pid* motors[8];
	CPGNode* cpg[4];

	double time = 0;
	unsigned long tick = 0;
  double pitch = 0;
  double roll = 0;
  double yaw = 0;
	double gimbal_lock_x = 0;
	double gimbal_lock_y = 0;

	double pitchVariance = 0;
	double rollVariance = 0;
	double yawVariance = 0;
	double xFinal = 0;
	double yFinal = 0;
	double zFinal = 0;

	bool touchedGround = false;
	bool leftGround = false;

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
	ofstream output;
};

#endif
