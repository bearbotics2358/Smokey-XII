#include <WPILib.h>
#include "ctre/Phoenix.h"
#include <SwerveModule.h>
#include <Prefs.h>

SwerveModule::SwerveModule(int driveMotorOne, int driveMotorTwo, int turnMotor):
a_DriveMotorOne(driveMotorOne),
a_DriveMotorTwo(driveMotorTwo),
a_TurnMotor(turnMotor)
{
	a_TurnMotor.ConfigSelectedFeedbackSensor(CTRE_MagEncoder_Relative, 0, 0);

}

void SwerveModule::UpdateRaw(float driveSpeed, float rotationSpeed)
{
	float scalar = 1.0; // Full Speed is 1.0

	a_DriveMotorOne.Set(scalar * driveSpeed); // Because this method is just for
	a_DriveMotorTwo.Set(scalar * driveSpeed); // mechanically testing the modules,
	a_TurnMotor.Set(scalar * rotationSpeed); // I've applied a scalar for safety.
}

void SwerveModule::UpdateSpeed(float driveSpeed)
{
	float scalar = 1.0; // Full Speed is 1.0
	a_DriveMotorOne.Set(scalar * driveSpeed);
	a_DriveMotorTwo.Set(scalar * driveSpeed);
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
	float counts = angle * (4096 / 360.0);
	a_TurnMotor.Set(ControlMode::Position, counts);
}

void SwerveModule::UpdateTraj(float deltaDist, float angle)
{

}

void SwerveModule::ZeroEncoders(void)
{
	a_DriveMotorOne.SetSelectedSensorPosition(0, 0, 0);
	a_TurnMotor.SetSelectedSensorPosition(0, 0, 0);
}

float SwerveModule::GetAngleRaw(void)
{
	float ret;
	ret = a_TurnMotor.GetSelectedSensorPosition(0);
	return ret;
}

float SwerveModule::GetAngle(void)
{
	float count;
	count = GetAngleRaw(); // Returns raw value from the encoder

	float ret = ((count / COUNTS_PER_ROTATION) * 360); // Rotations * Degrees per rotation

	ret = (-1 * (int) ret % 360); // Converts counts to int casts it between 0 and 360 degrees

	if(ret > 180) // Restricting 0 to 360 to between +/- 180
	{
		ret -= 360;
	}
	if(ret < -180)
	{
		ret += 360;
	}


	return ret;
}



float SwerveModule::GetDistanceRaw(void)
{
	float ret;
	ret = a_DriveMotorOne.GetSelectedSensorPosition(0);
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
	else if(id == FL_DRIVE_TWO_ID) {
	ret = a_DriveMotorTwo.GetOutputCurrent();
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
	else if(id == FL_DRIVE_TWO_ID) {
	ret = a_DriveMotorTwo.GetMotorOutputVoltage();
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

void SwerveModule::SetTurnPID(float p, float i, float d)
{
	a_TurnMotor.Config_kP(0, p, 0);
	a_TurnMotor.Config_kI(0, i, 0);
	a_TurnMotor.Config_kD(0, d, 0);
}

SwerveModule::~SwerveModule(void)
{

}
