
#ifndef PIDF_CONTROLLER_H
#define PIDF_CONTROLLER_H

class PIDFController // PID Controller class to allow us to do PID-based position/velocity closed loops
{ // NOTE: FIRST has a PIDController class so I had to name it PIDFController so it would compile correctly. Sorry!
    private:
        float pGain;
        float iGain;
        float dGain;
        float fGain; // While called a PID Controller, this class will also allow the use of an f gain eventually.

        float error; // Between (-1, 1), represents percent error
        float integral; // for I value
        float derivative; // for D value
        // TODO: Add f value

        float cycle;
    public:
        PIDFController(float p, float i, float d, float f);
        PIDFController(float p, float i, float d);
        ~PIDFController();
        
        // Cycle = value for 100% -> error = (current - target) / cycle;
        float GetOutput(void);
        float GetError(void);

        void Update(float target, float current); // Call every iteration to update error, integral and derivative values
        
        float GetCycle(void);
        void SetCycle(float inCycle);

        // float PIDKill(float target);
};

#endif