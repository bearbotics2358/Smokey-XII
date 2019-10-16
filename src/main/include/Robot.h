#include <frc/WPILib.h>
#include <SwerveModule.h>
#include <BeamBreak.h>
#include <CargoCollector.h>
#include <HatchCollector.h>
#include <SwerveDrive.h>
#include <JrimmyGyro.h>
#include <LineFollower.h>
#include "MQTTInterface.h"
#include <frc/CAN.h>
#include <LightRingController.h>
#include <PIDManager.h>
// #include <Gunnar.h>

#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

class Robot : public frc::TimedRobot
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
	int MBWOCFFHS(int targetAng); // MeBeingWorkingOnCodeForFiveHoursStraight
private:
	
	std::string robotState;
	bool cruiseControl;
	bool crabToggle;
	bool cargoToggle;
	bool cargoLastInput;
	bool cargoDirection;
	float driveSpeed;
	float rotationSpeed;
	int targetAngle;
	int clawCamCirco;
	int cargoCamCirco;
	int counter;

	JrimmyGyro a_Gyro;
	frc::Joystick a_Joystick1;
	frc::Joystick a_Controller1;
	SwerveModule FL_SwerveModule;
	SwerveModule FR_SwerveModule;
	SwerveModule BL_SwerveModule;
	SwerveModule BR_SwerveModule;
	SwerveDrive a_SwerveDrive;
	CargoCollector a_CargoCollector;
	HatchCollector a_HatchCollector;
	LineFollower a_Follower1;
	MQTTInterface a_Interface;
	LightRingController a_Light;
	PIDManager a_PIDManager;

};



#endif /* SRC_Robot_H_ */
