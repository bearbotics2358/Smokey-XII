
#ifndef SRC_PIDMANAGER_H
#define SRC_PIDMANAGER_H

#include <PIDFController.h>
#include <frc/WPILib.h>

class PIDManager 
{
    private:
        PIDFController a_AngleLockPID;
    public:
        PIDManager(void);
        float GetAngLock(void);
        void UpdateAngLock(float target, float current);
        void ResetAngLock(void);
};

#endif