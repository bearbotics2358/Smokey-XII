#include <frc/WPILib.h>
#include "ctre/Phoenix.h"
#include <SwerveModule.h>
#include <Prefs.h>

SwerveModule::SwerveModule(int driveMotorOne, int turnMotor, PIDManager *a_PIDManager):
a_DriveMotorOne(driveMotorOne),
a_TurnMotor(turnMotor)
{
	a_DriveMotorOne.ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);
	a_DriveMotorOne.ConfigFeedbackNotContinuous(false, 0);
	// a_DriveMotorOne.SetInverted(true);
	a_TurnMotor.ConfigFeedbackNotContinuous(false, 0);
	a_TurnMotor.ConfigSelectedFeedbackSensor(CTRE_MagEncoder_Relative, 0, 0);
}

void SwerveModule::UpdateRaw(float driveSpeed, float rotationSpeed)
{
	float scalar = 1.0; // Full Speed is 1.0

	a_DriveMotorOne.Set(scalar * driveSpeed); // Because this method is just for testing mechanisms
	a_TurnMotor.Set(scalar * rotationSpeed); // I've applied a scalar for safety.
}

void SwerveModule::UpdateSpeed(float driveSpeed)
{
	float scalar = 0.75; // Full Speed is 1.0
	a_DriveMotorOne.Set(scalar * driveSpeed);
}

void SwerveModule::UpdateSpeedPID(float driveSpeed) // Velocity PID-Based Closed Control Loop
{
	//    Rev     sec     COUNTS
	// 5 ----- * ----- *  -------
	//    Sec     10        Rev
	double scalar = -0.75;
	double speed = scalar * driveSpeed * ((5.0 / 10.0) * COUNTS_PER_DRIVE_ROTATION);
	a_DriveMotorOne.Set(ControlMode::Velocity, driveSpeed);
}

void SwerveModule::UpdateAngle(float desiredAngle) // -180 < angle < 180
{
	float currentAngle = GetAngle();

	//  Positive Motor Speed = Unit-Circle (counter-clockwise)
	float turnSpeed = 0.30;
	if(currentAngle < desiredAngle && currentAngle - desiredAngle > -180)
	{
		a_TurnMotor.Set(-turnSpeed);
	}
	else if(currentAngle < desiredAngle && currentAngle - desiredAngle < -180)
	{
		a_TurnMotor.Set(turnSpeed);
	}
	else if(currentAngle > desiredAngle && currentAngle - desiredAngle > 180)
	{
		a_TurnMotor.Set(-turnSpeed);
	}
	else if(currentAngle > desiredAngle && currentAngle - desiredAngle < 180)
	{
		a_TurnMotor.Set(turnSpeed);
	}
	else if(currentAngle - desiredAngle == 180 || currentAngle - desiredAngle == -180)
	{
		a_TurnMotor.Set(turnSpeed);
	}
}

void SwerveModule::UpdateAnglePID(float angle)
{
	if(abs(angle - GetAngle()) > 180) // find coterminal if distance is greater than 180
	{
		if((GetAngle() > 180 && angle >= 0)  || (GetAngle() > -180 && angle < 0))
		{
			angle+=360; 
		}
		else
		{
			angle -=360;
		}
	}
	int counts = angle * (COUNTS_PER_ROTATION / 360); // Goal for rotation
	int revolutions;

	if(GetAngleRaw() > 0)
	{
		revolutions = GetAngleRaw() / COUNTS_PER_ROTATION; // Uses integer divison to find revolutions
	}
	else // This if/else should handle the weird negative integer division in c++
	{
		revolutions = (-1 * GetAngleRaw()) / COUNTS_PER_ROTATION; 
		revolutions = -1 * revolutions;
	}

	int calculatedValue = counts + (revolutions * COUNTS_PER_ROTATION);
	// a_TurnMotor.Set(ControlMode::Position, calculatedValue);
}

void SwerveModule::UpdateTraj(float deltaDist, float angle)
{

}

void SwerveModule::ZeroEncoders(void)
{
	a_DriveMotorOne.SetSelectedSensorPosition(0, 0, 0);
	a_TurnMotor.SetSelectedSensorPosition(0, 0, 0);
}

int SwerveModule::GetAngleRaw(void)
{
	int ret;
	ret = a_TurnMotor.GetSelectedSensorPosition(0);
	return ret;
}

float SwerveModule::GetAngle(void)
{
	float count;
	count = GetAngleRaw(); // Returns raw value from the encoder

	float ret;
	if(count > 0)
	{
		ret = ((count / COUNTS_PER_ROTATION) * 360); // Rotations * Degrees per rotation
	}
	else
	{
		ret = -1 * ((-1 * count/COUNTS_PER_ROTATION) * 360);
	}
	if(ret > 0)
	{
		ret = ((int) ret % 360); // Converts counts to int casts it between 0 and 360 degrees
	}
	else
	{
		ret = -1 * ((int) abs(ret) % 360);
	}
	/*
	*     |-------0/360-------|
	*     |                   |
	*     |                   |
	*     90                 270
	*     |                   |
	*     |                   |
	*     |--------180--------|	
	*/
	/*if(ret > 180) // Restricting 0 to 360 to between +/- 180
	{
		ret -= 360;
	}
	if(ret < -180)
	{
		ret += 360;
	}
	*/

	return ret;
}



float SwerveModule::GetDistanceRaw(void)
{
	float ret;
	ret = -1 * a_DriveMotorOne.GetSelectedSensorPosition(0);
	return ret;
}

float SwerveModule::GetDistanceIn(void)
{
	float count;
	count = GetDistanceRaw();


	float ret = ((count / (COUNTS_PER_ROTATION * GEAR_RATIO_SCALAR)) * WHEEL_CIRCUM_IN);

	return ret;
}

float SwerveModule::GetDistanceCm(void)
{
	float count;
	count = GetDistanceRaw();


	float ret = ((count / (COUNTS_PER_ROTATION * GEAR_RATIO_SCALAR)) * WHEEL_CIRCUM_CM);

	return ret;
}

float SwerveModule::GetCurrentOP(int id)
{
	float ret;
	if(id == FL_DRIVE_ONE_ID) {
	ret = a_DriveMotorOne.GetOutputCurrent();
	}
	else if(id == FL_TURN_ID) {
		ret = a_TurnMotor.GetOutputCurrent();
	}
	else
	{
		ret = 0;
	}

	return ret;
}

float SwerveModule::GetVoltageOP(int id)
{
	float ret;
	if(id == FL_DRIVE_ONE_ID) {
	ret = a_DriveMotorOne.GetMotorOutputVoltage();
	}
	else if(id == FL_TURN_ID) {
		ret = a_TurnMotor.GetMotorOutputVoltage();
	}
	else
	{
		ret = 0;
	}

	return ret;
}

float SwerveModule::GetVelocity(void)
{
	return a_DriveMotorOne.GetSelectedSensorVelocity(0);
}

void SwerveModule::SetTurnPID(float p, float i, float d)
{
	a_TurnMotor.Config_kP(0, p, 0);
	a_TurnMotor.Config_kI(0, i, 0);
	a_TurnMotor.Config_kD(0, d, 0);
}

void SwerveModule::SetDrivePIDF(float p, float i, float d, float f)
{
	a_DriveMotorOne.Config_kP(0, p, 0);
	a_DriveMotorOne.Config_kI(0, i, 0);
	a_DriveMotorOne.Config_kD(0, d, 0);
	a_DriveMotorOne.Config_kF(0, f, 0);
}

SwerveModule::~SwerveModule(void)
{

}
