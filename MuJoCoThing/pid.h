#ifndef pid_h
#define pid_h
#endif

class pid{
    public:
        pid(double KP, double KI, double KD);
        double calculateOutput(double position, double setpoint);
		double calculateOutput(double position, double velocity, double setpoint);
    private:
        double KP;
        double KI;
        double KD;
        double lastError;
        double integral;
        double lastPosition;
};