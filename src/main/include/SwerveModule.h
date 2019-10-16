
#pragma once

#include <frc/WPILib.h>
#include "ctre/Phoenix.h"
#include "PIDFController.h"

#ifndef SRC_SWEREMODULE_H_
#define SRC_SWEREMODULE_H_

class SwerveModule { 
public:
	SwerveModule(int driveMotorOne, int turnMotor, PIDFController *controller); // Takes CAN IDs for the different talons so this class can be reused for more than one module.

	void UpdateSpeed(float driveSpeed);
	void UpdateSpeedPID(float driveSpeed);

	void UpdateRaw(float driveSpeed, float rotationSpeed); // Method for mechanically testing swerve modules, will literally set the drive talons to a specific speed.
	void UpdateAngle(float desiredAngle); // Angle in degrees
	void UpdateAnglePID(float angle);
	void UpdateAngleNewPID(float angle);
	void UpdateTraj(float deltaDist, float angle);

	void ZeroEncoders(void);
	int GetAngleRaw(void);
	float GetAngle(void);

	float GetDistanceRaw(void);
	float GetDistanceIn(void);
	float GetDistanceCm(void);

	float GetCurrentOP(int id);
	float GetVoltageOP(int id);

	float GetVelocity(void);

	void SetTurnPID(float p, float i, float d);
	void SetDrivePIDF(float p, float i, float d, float f);


	~SwerveModule(void);
private:
	WPI_TalonSRX a_DriveMotorOne;
	WPI_TalonSRX a_TurnMotor;

	PIDFController *a_PIDController;
};



#endif /* SRC_SWEREMODULE_H_ */
