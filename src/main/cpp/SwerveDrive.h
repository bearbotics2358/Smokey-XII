
#ifndef SRC_SWERVEDRIVE_H_
#define SRC_SWERVEDRIVE_H_

#include <WPILib.h>
#include "ctre\Phoenix.h"
#include <SwerveModule.h>

class SwerveDrive
{
public:

	SwerveDrive(void);
	~SwerveDrive(void);

	void CrabDrive(double xIn, double yIn, double zIn); // Crab - Can drive in any X-Y Direction
	void CrabDrivePID(double xIn, double yIn, double zIn); // (No Rotation)
	void CrabGyro(double xIn, double yIn, double zIn, double gyroValue);
private:
	SwerveModule FL_SwerveModule;
	SwerveModule FR_SwerveModule;
	SwerveModule BL_SwerveModule;
	SwerveModule BR_SwerveModule;
};

#endif /* SRC_SWERVEDRIVE_H_ */
