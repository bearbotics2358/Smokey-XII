
#include <PIDManager.h>
#include <Prefs.h>

PIDManager::PIDManager(void):
a_AngleLockPID(LOCK_P_GAIN, LOCK_I_GAIN, LOCK_D_GAIN, LOCK_F_GAIN)
{
    a_AngleLockPID.SetCycle(180);
}

float PIDManager::GetAngLock(void)
{
    return a_AngleLockPID.GetOutput();
}

void PIDManager::UpdateAngLock(float target, float current) // Current: raw gyro angle
{
    // ------0/360------
	// |               |   
 	// |               |
	// 90             270
	// |               |
	// |               |
	// --------180-----|

	if(current < 0)
		current = 360 - ((int) (-1 * current) % 360); // Limits 
	else
		current = (int) current % 360;

    if(abs(target - current) >= 180)
	{
		if(current > 180)
		{
			target+=360; 
		}
		else
		{
			target -=360;
		}
	}
    a_AngleLockPID.Update(target, current);
}