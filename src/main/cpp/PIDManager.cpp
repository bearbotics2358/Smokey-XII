
#include <PIDManager.h>
#include <Prefs.h>

PIDManager::PIDManager(void): // consider making pid loops public so they can actually be used.
a_AngleLockPID(LOCK_P_GAIN, LOCK_I_GAIN, LOCK_D_GAIN, LOCK_F_GAIN),
a_WheelFL(FL_TURN_P, FL_TURN_I, FL_TURN_D, 0),
a_WheelFR(FR_TURN_P, FR_TURN_I, FR_TURN_D, 0),
a_WheelBL(BL_TURN_P, BL_TURN_I, BL_TURN_D, 0),
a_WheelBR(BR_TURN_P, BR_TURN_I, BR_TURN_D, 0)
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

void PIDManager::ResetAngLock(void)
{
    a_AngleLockPID.ResetError();
}