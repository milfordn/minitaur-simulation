#ifndef CPGCTRL_H
#define CPGCTRL_H
#include "../Controller.h"
#include "../CPGNode.h"
#include "../pid.h"
class CPGController : public Controller{
  public:
    CPGController(double params[28]);
    void step(double dt) override;
  private:
    CPGNode* frontRight;
    CPGNode* frontLeft;
    CPGNode* backRight;
    CPGNode* backLeft;

    pid* motors[8];
    char* motorNames[8];

    double time = 0;
};


#endif
