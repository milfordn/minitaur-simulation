#ifndef pid_h
#define pid_h

class pid{
    public:
        pid(double KP, double KI, double KD);
        double calculateOutput(unsigned long tick, double setpoint, double position);
        double calculateOutput(unsigned long tick, double setpoint, double position, double velocity);
        void setPgain(double p);

    private:
        double KP;
        double KI;
        double KD;
        double lastError;
		double lastTick;
        double integral;
        double lastPosition;
};
#endif
