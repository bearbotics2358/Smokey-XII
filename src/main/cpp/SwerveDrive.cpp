
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
// a_Feather(1)
{
	integral = 0;
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
	bool inDeadzone = false;
	if(radius < DEADZONE)
	{
		inDeadzone = true;
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
	double scalar = 0.75;
	FR_Speed *= scalar;
    FL_Speed *= scalar;
    BR_Speed *= scalar;
    BL_Speed *= scalar;

	float currentFL = FL_SwerveModule.GetAngle(); 
	float currentFR = FR_SwerveModule.GetAngle();
	float currentBR = BR_SwerveModule.GetAngle(); 
	float currentBL = BL_SwerveModule.GetAngle();

	if(NeedsAngOpt(currentFL, FL_Angle))
	{
		if(FL_Angle < 0)
			FL_Angle += 180;
		else
			FL_Angle -= 180;
		FL_Speed *= -1;
	}
	if(NeedsAngOpt(currentFR, FR_Angle))
	{
		if(FR_Angle < 0)
			FR_Angle += 180;
		else
			FR_Angle -= 180;
		FR_Speed *= -1;
	}
	if(NeedsAngOpt(currentBR, BR_Angle))
	{
		if(BR_Angle < 0)
			BR_Angle += 180;
		else
			BR_Angle -= 180;
		BR_Speed *= -1;
	}
	if(NeedsAngOpt(currentBL, BL_Angle))
	{
		if(BL_Angle < 0)
			BL_Angle += 180;
		else
			BL_Angle -= 180;
		BL_Speed *= -1;
	}

	if(inDeadzone && zIn == 0)
	{
		FR_Speed = 0;
		FL_Speed = 0;
		BR_Speed = 0;
		BL_Speed = 0;

		FR_Angle = FR_SwerveModule.GetAngle();
		FL_Angle = FL_SwerveModule.GetAngle();
		BR_Angle = BR_SwerveModule.GetAngle();
		BL_Angle = BL_SwerveModule.GetAngle();
	}
	// FR_SwerveModule.UpdateSpeedPID(FR_Speed);
	FR_SwerveModule.UpdateSpeed(FR_Speed);
	FR_SwerveModule.UpdateAnglePID(FR_Angle);
	frc::SmartDashboard::PutNumber("FR Angle: ", FR_Angle);
	// FR_SwerveModule.UpdateAngle(FR_Angle);

	// FL_SwerveModule.UpdateSpeedPID(FL_Speed);
 	FL_SwerveModule.UpdateSpeed(FL_Speed);
	FL_SwerveModule.UpdateAnglePID(FL_Angle);
	frc::SmartDashboard::PutNumber("FL Angle: ", FL_Angle);
	// FL_SwerveModule.UpdateAngle(FL_Angle);

	// BL_SwerveModule.UpdateSpeedPID(BL_Speed);
	BL_SwerveModule.UpdateSpeed(BL_Speed);
	BL_SwerveModule.UpdateAnglePID(BL_Angle);
	frc::SmartDashboard::PutNumber("BL Angle: ", BL_Angle);
	// BL_SwerveModule.UpdateAngle(BL_Angle);

	// BR_SwerveModule.UpdateSpeedPID(BR_Speed);
	BR_SwerveModule.UpdateSpeed(BR_Speed);
	BR_SwerveModule.UpdateAnglePID(BR_Angle);
	frc::SmartDashboard::PutNumber("BR Angle: ", BR_Angle);
	// BR_SwerveModule.UpdateAngle(BR_Angle);

}

void SwerveDrive::SwerveRobotOriented(double xIn, double yIn, double zIn)
{
	double xInput = xIn;
	double yInput = yIn;
	double zInput = zIn;

	// Atan2() returns the angle in radians so we convert it to degrees.
	// double theta = (atan2(xInput, yInput)) * 180 / PI; // These two lines convert cartesian
	double radius = sqrt(pow(xInput, 2) + pow(yInput, 2));  // to polar coords
	bool inDeadzone = false;
	if(radius < DEADZONE)
	{
		inDeadzone = true;
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
	double scalar = 0.75;
	FR_Speed *= scalar;
    FL_Speed *= scalar;
    BR_Speed *= scalar;
    BL_Speed *= scalar;

	float currentFL = FL_SwerveModule.GetAngle(); 
	float currentFR = FR_SwerveModule.GetAngle();
	float currentBR = BR_SwerveModule.GetAngle(); 
	float currentBL = BL_SwerveModule.GetAngle();

	if(NeedsAngOpt(currentFL, FL_Angle))
	{
		if(FL_Angle < 0)
			FL_Angle += 180;
		else
			FL_Angle -= 180;
		FL_Speed *= -1;
	}
	if(NeedsAngOpt(currentFR, FR_Angle))
	{
		if(FR_Angle < 0)
			FR_Angle += 180;
		else
			FR_Angle -= 180;
		FR_Speed *= -1;
	}
	if(NeedsAngOpt(currentBR, BR_Angle))
	{
		if(BR_Angle < 0)
			BR_Angle += 180;
		else
			BR_Angle -= 180;
		BR_Speed *= -1;
	}
	if(NeedsAngOpt(currentBL, BL_Angle))
	{
		if(BL_Angle < 0)
			BL_Angle += 180;
		else
			BL_Angle -= 180;
		BL_Speed *= -1;
	}

	if(inDeadzone && zIn == 0)
	{
		FR_Speed = 0;
		FL_Speed = 0;
		BR_Speed = 0;
		BL_Speed = 0;

		FR_Angle = FR_SwerveModule.GetAngle();
		FL_Angle = FL_SwerveModule.GetAngle();
		BR_Angle = BR_SwerveModule.GetAngle();
		BL_Angle = BL_SwerveModule.GetAngle();
	}


	// FR_SwerveModule.UpdateSpeedPID(FR_Speed);
	FR_SwerveModule.UpdateSpeed(FR_Speed);
	FR_SwerveModule.UpdateAnglePID(FR_Angle);
	frc::SmartDashboard::PutNumber("FR Angle: ", FR_Angle);
	// FR_SwerveModule.UpdateAngle(FR_Angle);

	// FL_SwerveModule.UpdateSpeedPID(FL_Speed);
 	FL_SwerveModule.UpdateSpeed(FL_Speed);
	FL_SwerveModule.UpdateAnglePID(FL_Angle);
	frc::SmartDashboard::PutNumber("FL Angle: ", FL_Angle);
	// FL_SwerveModule.UpdateAngle(FL_Angle);

	// BL_SwerveModule.UpdateSpeedPID(BL_Speed);
	BL_SwerveModule.UpdateSpeed(BL_Speed);
	BL_SwerveModule.UpdateAnglePID(BL_Angle);
	frc::SmartDashboard::PutNumber("BL Angle: ", BL_Angle);
	// BL_SwerveModule.UpdateAngle(BL_Angle);

	// BR_SwerveModule.UpdateSpeedPID(BR_Speed);
	BR_SwerveModule.UpdateSpeed(BR_Speed);
	BR_SwerveModule.UpdateAnglePID(BR_Angle);
	frc::SmartDashboard::PutNumber("BR Angle: ", BR_Angle);
	// BR_SwerveModule.UpdateAngle(BR_Angle);

}

void SwerveDrive::MakeshiftRotate(float input)
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

void SwerveDrive::SetRobotAngle(float target, float current)
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

	float pGain = 1.75;
	float iGain = 0.75;

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
	float error = (current - target) / 360.0; // Positive should be counter-clockwise
	integral += (error * 0.02);
	float outputMax = 0.75;
	float forJason = outputMax * (pGain * (error) + (iGain * integral)); // P * error + I * intergral + D * derivative
	MakeshiftRotate(forJason); // TODO: Finish this and test it
}

void SwerveDrive::AngleLock(float xIn, float yIn, float target, float gyroValue, bool fieldOriented)
{
	// ------0/360------
	// |               |   
 	// |               |
	// 90             270
	// |               |
	// |               |
	// --------180-----|

	float current = gyroValue;

	if(current < 0)
		current = 360 - ((int) (-1 * current) % 360); // Limits 
	else
		current = (int) current % 360;

	float pGain = 2.25;
	float iGain = 0.5;

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
	float error = (current - target) / 180; // Positive should be counter-clockwise
	integral += (error * 0.02);
	float outputMax = 0.75;
	float forJason = outputMax * (pGain * (error) + (iGain * integral)); // P * error + I * intergral + D * derivative

	double radius = sqrt(pow(xIn, 2) + pow(yIn, 2));  // to polar coords
	if(radius < DEADZONE)
		forJason = 0;

	if(fieldOriented)
		SwerveDriveUpdate(xIn, yIn, -forJason, gyroValue);
	else
		SwerveRobotOriented(xIn, yIn, -forJason);
}

void SwerveDrive::DriveDistanceRaw(float target, float gyroValue)
{
	float average = (FL_SwerveModule.GetDistanceIn() + FR_SwerveModule.GetDistanceIn() + BL_SwerveModule.GetDistanceIn() + BR_SwerveModule.GetDistanceIn())/4.0;
	if(average >= target)
	{
		SwerveRobotOriented(0, 0, 0);
	}
	else
	{
		AngleLock(0, 0.3, 0, gyroValue, false); // Gyro value might matter not sure yet?
	} // I think it does
}// Ok it probably does

float SwerveDrive::XForCenter(float current)
{
	/* FL_SwerveModule.UpdateAnglePID(90);
	FR_SwerveModule.UpdateAnglePID(90);
	BL_SwerveModule.UpdateAnglePID(90);
	BR_SwerveModule.UpdateAnglePID(90);

	float tempAngle = FL_SwerveModule.GetAngle();
	if(tempAngle < 0)
		tempAngle = 360 - ((int) (-1 * tempAngle) % 360); // Limits 
	else
		tempAngle = (int) tempAngle % 360;

	if(tempAngle - 90 < 5 || tempAngle - 90 > -5)
	{
		*/
	float pGain = 0.6;

	float error = (-current / 5.0); 
	float outputMax = 0.5;
	float forJason = outputMax * (pGain * (error)); // P * error + I * intergral + D * derivative

	return forJason;
}

float SwerveDrive::VisionZAxis(float visionError)
{
	float pGain = 0.7;
	float outputMax = 0.65;

	float forKJason = (outputMax * (pGain * (-visionError / 10.0)));
	return forKJason;
}

bool SwerveDrive::NeedsAngOpt(float current, float target)
{
	float currentFL = current;
	float FL_Angle = target;
	if(currentFL < 360 && currentFL > 180)
		currentFL -= 360;
	else if(currentFL > -360 && currentFL < -180)
		currentFL += 360;

	
	//           0
	// 
	// 90                -90
	//	       
	//       180 or -180

	
	if(fabs(currentFL) < 90 && fabs(FL_Angle) < 90)
	{
		if(fabs(currentFL - FL_Angle) > 90)
		{
			return true;
		}
	}
	else if((fabs(currentFL) >= 90 && fabs(FL_Angle) < 90) || (fabs(FL_Angle) >= 90 && fabs(currentFL) < 90))
	{
		if(fabs(currentFL - FL_Angle) > 90)
		{
			return true;
		}
	}
	else
	{
		if(currentFL < 0 && FL_Angle > 0)
		{
			if(fabs((360 + currentFL) - FL_Angle) > 90)
			{
				return true;
			}
		} 
		else if(currentFL > 0 && FL_Angle < 0)
		{
			if(fabs((360 + FL_Angle) - currentFL) > 90)
			{
				return true;
			}
		}
	}
		/*
		if(FL_Angle < 0)
			FL_Angle += 180;
		else
			FL_Angle -= 180;
		FL_Speed *= -1;
		*/
	return false;
}