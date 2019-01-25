
#include <WPILib.h>
#include "ctre\Phoenix.h"
#include <SwerveDrive.h>
#include <SwerveModule.h>
#include <Prefs.h>

SwerveDrive::SwerveDrive(void):
	FL_SwerveModule(FL_DRIVE_ONE_ID, FL_TURN_ID),
	FR_SwerveModule(FR_DRIVE_ONE_ID, FR_TURN_ID),
	BL_SwerveModule(BL_DRIVE_ONE_ID, BL_TURN_ID),
	BR_SwerveModule(BR_DRIVE_ONE_ID, BR_TURN_ID)
	{
	}

SwerveDrive::~SwerveDrive(void)
{

}

void SwerveDrive::CrabDrive(double xIn, double yIn, double zIn)
{
	double xInput = xIn;
	double yInput = -1.0 * yIn;
	// double zInput;`


	// Atan2() returns the angle in radians so we convert it to degrees.
	double theta = (atan2(xInput, yInput)) * 180 / PI; // These two lines convert cartesian
	double radius = sqrt(pow(xInput, 2) + pow(yInput, 2));  // to polar coords

	if(radius < 0.42)
	{
		theta = 0;
		radius = 0;
	}

	if(radius > 1.0)
		{
			radius = 1.0; // Makes sure magnitude doesn't go over 1
		}
	radius = radius * 0.15; // For testing purposes, we will scale the input

	// SmartDashboard::PutNumber("Theta: ", theta);
	// SmartDashboard::PutNumber("Radius: ", radius);
	FL_SwerveModule.UpdateSpeed(radius);
	FL_SwerveModule.UpdateAngle(theta);

	FR_SwerveModule.UpdateSpeed(radius);
	FR_SwerveModule.UpdateAngle(theta);

	BL_SwerveModule.UpdateSpeed(radius);
	BL_SwerveModule.UpdateAngle(theta);

	BR_SwerveModule.UpdateSpeed(radius);
	BR_SwerveModule.UpdateAngle(theta);
}

void SwerveDrive::CrabDrivePID(double xIn, double yIn, double zIn)
{
	double xInput = xIn;
	double yInput = yIn;
	// double zInput;


	// Atan2() returns the angle in radians so we convert it to degrees.
	double theta = (atan2(xInput, yInput)) * 180 / PI; // These two lines convert cartesian
	double radius = sqrt(pow(xInput, 2) + pow(yInput, 2));  // to polar coords

	if(radius < 0.42)
	{
		theta = 0;
		radius = 0;
	}

	if(radius > 1.0)
		{
			radius = 1.0; // Makes sure magnitude doesn't go over 1
		}
	radius = radius * -1.00; // For testing purposes, we will scale the input

	// SmartDashboard::PutNumber("Theta: ", theta);
	// SmartDashboard::PutNumber("Radius: ", radius);

	FL_SwerveModule.UpdateSpeed(radius);
	FL_SwerveModule.UpdateAnglePID(theta);
	// FL_SwerveModule.SetTurnPID(10, 0, 0.5);

	FR_SwerveModule.UpdateSpeed(radius);
	FR_SwerveModule.UpdateAnglePID(theta);
	// FR_SwerveModule.SetTurnPID(10, 0, 0.5);

	BL_SwerveModule.UpdateSpeed(radius);
	BL_SwerveModule.UpdateAnglePID(theta);
	// BL_SwerveModule.SetTurnPID(10, 0, 0.5);

	BR_SwerveModule.UpdateSpeed(radius);
	BR_SwerveModule.UpdateAnglePID(theta);
	 // BR_SwerveModule.SetTurnPID(10, 0, 0.5);
}

void SwerveDrive::CrabGyro(double xIn, double yIn, double zIn, double gyroValue)
{
	double xInput = xIn;
	double yInput = yIn;
	// double zInput;


	// Atan2() returns the angle in radians so we convert it to degrees.
	double theta = (atan2(xInput, yInput)) * 180 / PI; // These two lines convert cartesian
	double radius = sqrt(pow(xInput, 2) + pow(yInput, 2));  // to polar coords

	if(radius < 0.42)
	{
		theta = 0;
		radius = 0;
	}

	if(radius > 1.0)
		{
			radius = 1.0; // Makes sure magnitude doesn't go over 1
		}
	radius = radius * -1.00; // For testing purposes, we will scale the input

	// SmartDashboard::PutNumber("Theta: ", theta);
	// SmartDashboard::PutNumber("Radius: ", radius);

	FL_SwerveModule.UpdateSpeed(radius);
	FL_SwerveModule.UpdateAnglePID(theta);

	FR_SwerveModule.UpdateSpeed(radius);
	FR_SwerveModule.UpdateAnglePID(theta);

	// BL_SwerveModule.UpdateSpeed(radius);
	// BL_SwerveModule.UpdateAnglePID(theta);

	// BR_SwerveModule.UpdateSpeed(radius);
	// BR_SwerveModule.UpdateAnglePID(theta);
}
