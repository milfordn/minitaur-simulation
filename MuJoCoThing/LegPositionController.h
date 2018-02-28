#include "pid.h"

class LegPositionController{
public :
	LegPositionController(int m1, int m2, int m1Angle, int m2Angle, int end);
	~LegPositionController();
	void step(struct _mjData *, struct _mjModel *);
	void setLength(double);
	void setAngle(double);
	void release(struct _mjData *);
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
	
	double desiredLength;
	double desiredAngle;
	
	const double L1 = 0.1; //Length of thigh
	const double L2 = 0.2; //Length of shin
};

