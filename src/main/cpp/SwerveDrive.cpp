
#include <frc/WPILib.h>
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

	if(radius < DEADZONE)
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
	// FL_SwerveModule.UpdateSpeed(radius);
	FL_SwerveModule.UpdateAngle(theta);

	// FR_SwerveModule.UpdateSpeed(radius);
	FR_SwerveModule.UpdateAngle(theta);

	// BL_SwerveModule.UpdateSpeed(radius);
	BL_SwerveModule.UpdateAngle(theta);

	// BR_SwerveModule.UpdateSpeed(radius);
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

	if(theta <= 0)
	{
		theta+=360;
	}

	if(radius < DEADZONE) // Dead zone
	{
		theta = 0;
		radius = 0;
	}

	if(radius > 1.0)
		{
			radius = 1.0; // Makes sure magnitude doesn't go over 1
		}
	radius = radius * 0.20; // For testing purposes, we will scale the input

	frc::SmartDashboard::PutNumber("Theta: ", theta);
	// SmartDashboard::PutNumber("Radius: ", radius);

	// FL_SwerveModule.UpdateSpeedPID(radius*500.0 * 4096 / 600);
	// FL_SwerveModule.UpdateSpeed(radius);
	FL_SwerveModule.UpdateAnglePID(theta);

	// FR_SwerveModule.UpdateSpeedPID(radius*500.0 * 4096 / 600);
	// FR_SwerveModule.UpdateSpeed(radius);
	FR_SwerveModule.UpdateAnglePID(theta);

	// BL_SwerveModule.UpdateSpeedPID(radius*500.0 * 4096 / 600);
	// BL_SwerveModule.UpdateSpeed(radius);
	BL_SwerveModule.UpdateAnglePID(theta);

	// BR_SwerveModule.UpdateSpeedPID(radius*500.0 * 4096 / 600);
	// BR_SwerveModule.UpdateSpeed(radius);
	BR_SwerveModule.UpdateAnglePID(theta);
}

void SwerveDrive::CrabGyro(double xIn, double yIn, double zIn, double gyroValue)
{
	double xInput = xIn;
	double yInput = yIn;
	// double zInput;

	// MAKES ROBOT "FIELD ORIENTED"???
	float gyroRadians = gyroValue * PI / 180; // converts gyro degrees to radians
	float temp = yInput * cos(gyroRadians) + xInput * sin(gyroRadians);
	xInput = -yInput * sin(gyroRadians) + xInput * cos(gyroRadians);
	yInput = temp;

	// Atan2() returns the angle in radians so we convert it to degrees.
	double theta = (atan2(xInput, yInput)) * 180 / PI; // These two lines convert cartesian
	double radius = sqrt(pow(xInput, 2) + pow(yInput, 2));  // to polar coords

	if(radius < DEADZONE)
	{
		theta = 0;
		radius = 0;
	}

	if(radius > 1.0)
		{
			radius = 1.0; // Makes sure magnitude doesn't go over 1
		}
	radius = 0.15 * radius; // For testing purposes, we will scale the input

	frc::SmartDashboard::PutNumber("Theta: ", theta);
	// SmartDashboard::PutNumber("Radius: ", radius);

	FL_SwerveModule.UpdateSpeedPID(radius*500.0 * 4096 / 600);
	// FL_SwerveModule.UpdateSpeed(radius);
	FL_SwerveModule.UpdateAnglePID(theta);

	FR_SwerveModule.UpdateSpeedPID(radius*500.0 * 4096 / 600);
	// FR_SwerveModule.UpdateSpeed(radius);
	FR_SwerveModule.UpdateAnglePID(theta);

	BL_SwerveModule.UpdateSpeedPID(radius*500.0 * 4096 / 600);
	// BL_SwerveModule.UpdateSpeed(radius);
	BL_SwerveModule.UpdateAnglePID(theta);

	BR_SwerveModule.UpdateSpeedPID(radius*500.0 * 4096 / 600);
	// BR_SwerveModule.UpdateSpeed(radius);
	BR_SwerveModule.UpdateAnglePID(theta);
}

void SwerveDrive::SwerveDriveUpdate(double xIn, double yIn, double zIn, double gyroValue)
{
	double xInput = xIn;
	double yInput = yIn;
	double zInput = zIn;

	// MAKES ROBOT "FIELD ORIENTED"???
	float gyroRadians = gyroValue * PI / 180; // converts gyro degrees to radians
	float temp = yInput * cos(gyroRadians) + xInput * sin(gyroRadians);
	xInput = -yInput * sin(gyroRadians) + xInput * cos(gyroRadians);
	yInput = temp;

	// Atan2() returns the angle in radians so we convert it to degrees.
	// double theta = (atan2(xInput, yInput)) * 180 / PI; // These two lines convert cartesian
	double radius = sqrt(pow(xInput, 2) + pow(yInput, 2));  // to polar coords

	if(radius < DEADZONE)
	{
		xInput = 0;
		yInput = 0;	
	}

	double r =  sqrt((DRIVE_TRAIN_FRONT_TO_BACK * DRIVE_TRAIN_FRONT_TO_BACK) + (DRIVE_TRAIN_SIDE_TO_SIDE * DRIVE_TRAIN_SIDE_TO_SIDE));

	double a = xInput - zInput * (DRIVE_TRAIN_FRONT_TO_BACK / r);
	double b = xInput + zInput * (DRIVE_TRAIN_FRONT_TO_BACK / r);
	double c = yInput - zInput * (DRIVE_TRAIN_SIDE_TO_SIDE / r);
	double d = yInput + zInput * (DRIVE_TRAIN_SIDE_TO_SIDE / r);

	double FL_Speed = sqrt(b*b + c*c);
    double FR_Speed = sqrt(b*b + d*d);
    double BL_Speed = sqrt(a*a + d*d);
    double BR_Speed = sqrt(a*a + c*c);

	double FL_Angle = atan2(b,c) * 180/PI;
    double FR_Angle = atan2(b,d) * 180/PI;
    double BR_Angle = atan2(a,d) * 180/PI;
    double BL_Angle = atan2(a,c) * 180/PI;

	double max = std::max(std::max(FR_Speed, FL_Speed), std::max(BR_Speed, BL_Speed));

    if(max > 1)
	{
    	FR_Speed /= max;
    	FL_Speed /= max;
    	BR_Speed /= max;
    	BL_Speed /= max;
    }
	double scalar = 0.85;
	FR_Speed *= scalar;
    FL_Speed *= scalar;
    BR_Speed *= scalar;
    BL_Speed *= scalar;

	// FR_SwerveModule.UpdateSpeedPID(radius*500.0 * 4096 / 600);
	FR_SwerveModule.UpdateSpeed(FR_Speed);
	FR_SwerveModule.UpdateAnglePID(FR_Angle);

	// FL_SwerveModule.UpdateSpeedPID(radius*500.0 * 4096 / 600);
 	FL_SwerveModule.UpdateSpeed(FL_Speed);
	FL_SwerveModule.UpdateAnglePID(FL_Angle);

	// BL_SwerveModule.UpdateSpeedPID(radius*500.0 * 4096 / 600);
	BL_SwerveModule.UpdateSpeed(BL_Speed);
	BL_SwerveModule.UpdateAnglePID(BL_Angle);

	// BR_SwerveModule.UpdateSpeedPID(radius*500.0 * 4096 / 600);
	BR_SwerveModule.UpdateSpeed(BR_Speed);
	BR_SwerveModule.UpdateAnglePID(BR_Angle);
}

void SwerveDrive::MakeshiftRotate(double input)
{

	FL_SwerveModule.UpdateAnglePID(-45);
	BL_SwerveModule.UpdateAnglePID(45);
	FR_SwerveModule.UpdateAnglePID(-135);
	BR_SwerveModule.UpdateAnglePID(135);

	FL_SwerveModule.UpdateSpeed(input);
	BL_SwerveModule.UpdateSpeed(input);
	FR_SwerveModule.UpdateSpeed(input);
	BR_SwerveModule.UpdateSpeed(input);
}
