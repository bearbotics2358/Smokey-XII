#include <WPILib.h>
#include <SwerveModule.h>
#include <SwerveDrive.h>
#include <JrimmyGyro.h>

#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

class Robot : public frc::IterativeRobot
{
public:
	Robot(void);

	void RobotInit(void);
	void RobotPeriodic(void);

	void DisabledInit(void);
	void DisabledPeriodic(void);

	void TeleopInit(void);
	void TeleopPeriodic(void);

	void AutonomousInit(void);
	void AutonomousPeriodic(void);

	void TestInit(void);
	void TestPeriodic(void);

private:
	std::string robotState;
	bool cruiseControl;
	bool crabToggle;
	float driveSpeed;
	float rotationSpeed;

	JrimmyGyro a_Gyro;
	frc::Joystick a_Joystick1;
	SwerveModule FL_SwerveModule;
	SwerveModule FR_SwerveModule;
	SwerveDrive a_SwerveDrive;


};



#endif /* SRC_Robot_H_ */
