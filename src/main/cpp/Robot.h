#include <frc/WPILib.h>
#include <SwerveModule.h>
#include <BeamBreak.h>
#include <SwerveDrive.h>
#include <JrimmyGyro.h>
#include <frc/CAN.h>

#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_


#define CAN_TEST


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

#ifndef CAN_TEST

	JrimmyGyro a_Gyro;
	frc::Joystick a_Joystick1;
	SwerveModule FL_SwerveModule;
	SwerveModule FR_SwerveModule;
	SwerveModule BL_SwerveModule;
	SwerveModule BR_SwerveModule;
	SwerveDrive a_SwerveDrive;
	BeamBreak a_BeamBreak;

#endif

	frc::CAN a_FeatherOne;


};



#endif /* SRC_Robot_H_ */
