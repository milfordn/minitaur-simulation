#include "../pid.h"
#ifndef LEG_CONTROLLER_H
#define LEG_CONTROLLER_H
class LegPositionController{
public :
	LegPositionController(int m1, int m2, int m1Angle, int m2Angle, int end);
	~LegPositionController();
	void step(struct _mjData *, struct _mjModel *);
	void setLength(double);
	void setAngle(double);
	void setPgain(double);
	void release();
	double getLength();
	double getAngle();
private:
	unsigned long tick;
	pid *m1; //Torque-level PID controller for controlling angle of motor 1
	pid *m2; //Torque-level PID controller for controlling angle of motor 2
	int m1ID;
	int m2ID;
	int m1Angle;
	int m2Angle;
	int m1JointID;
	int m2JointID;
	int endID;
	bool active = true;
	double desiredLength = .173205;
	double desiredAngle = 1.5708;
	double currentAngle;
	double currentLength;
	double L1;
	double L2;
};
#endif
