#include <WPILib.h>
#include "ctre/Phoenix.h"

#ifndef SRC_SWEREMODULE_H_
#define SRC_SWEREMODULE_H_

class SwerveModule {
public:
	SwerveModule(int driveMotorOne, int driveMotorTwo, int turnMotor); // Takes CAN IDs for the different talons so this class can be reused for more than one module.

	void UpdateSpeed(float driveSpeed);
	void UpdateRaw(float driveSpeed, float rotationSpeed); // Method for mechanically testing swerve modules, will literally set the drive talons to a specific speed.
	void UpdateAngle(float desiredAngle); // Angle in degrees
	void UpdateAnglePID(float angle);
	void UpdateTraj(float deltaDist, float angle);

	void ZeroEncoders(void);
	float GetAngleRaw(void);
	float GetAngle(void);

	float GetDistanceRaw(void);
	float GetDistanceIn(void);
	float GetDistanceCm(void);

	float GetCurrentOP(int id);
	float GetVoltageOP(int id);

	void SetTurnPID(float p, float i, float d);

	~SwerveModule(void);
private:
	WPI_TalonSRX a_DriveMotorOne;
	WPI_TalonSRX a_DriveMotorTwo;
	WPI_TalonSRX a_TurnMotor;

};



#endif /* SRC_SWEREMODULE_H_ */
