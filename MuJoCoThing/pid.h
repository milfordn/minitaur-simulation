#ifndef pid_h
#define pid_h

class pid{
    public:
        pid(double KP, double KI, double KD);
        double calculateOutput(double tick, double setpoint, double position);
		    double calculateOutput(double tick, double setpoint, double position, double velocity);
		    void setPgain(double p);
        void limitOutput(double l, double u);
    private:
        double KP;
        double KI;
        double KD;
        double lastError;
		    double lastTick;
        double integral;
        double lastPosition;
        double upper, lower;
        double limit(double outputer);
};
#endif
