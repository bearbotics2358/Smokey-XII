
#pragma once

#include <PIDFController.h>
#include <frc/WPILib.h>

class PIDManager 
{
    private:
        PIDFController a_AngleLockPID;
    public:
        PIDFController a_WheelFL;
        PIDFController a_WheelFR;
        PIDFController a_WheelBL;
        PIDFController a_WheelBR;

        PIDManager(void);
        float GetAngLock(void);
        void UpdateAngLock(float target, float current);
        void ResetAngLock(void);
};

