#include <frc/WPILib.h> // <WPILib.h> is deprecated
#include <Prefs.h>
#include <Robot.h>
// o(╥﹏╥)o
// (>-.-)>-)====>
Robot::Robot(void):
a_Gyro(frc::I2C::kMXP),
a_Joystick1(JOYSTICK_PORT_ONE),
a_Controller1(CONTROLLER_PORT_ONE),
a_CargoCollector(),
a_HatchCollector(),
FL_SwerveModule(FL_DRIVE_ONE_ID, FL_TURN_ID),
FR_SwerveModule(FR_DRIVE_ONE_ID, FR_TURN_ID),
BL_SwerveModule(BL_DRIVE_ONE_ID, BL_TURN_ID),
BR_SwerveModule(BR_DRIVE_ONE_ID, BR_TURN_ID),
a_SwerveDrive(),
// a_Gunnar("RIOclient", "localhost", 1183),
a_Follower1(1)
{
	// (>>'-')>>	
	a_Gyro.Init();
	cruiseControl = false;
	crabToggle = false;
	cargoToggle = false;
	cargoLastInput = false;
	cargoDirection = true;
	driveSpeed = 0;
	rotationSpeed = 0;
	targetAngle = -999;
	frc::SmartDashboard::init();
}

void Robot::RobotInit(void)
{
	
	/*FL_SwerveModule.ZeroEncoders();
	FR_SwerveModule.ZeroEncoders();
	BL_SwerveModule.ZeroEncoders();
	BR_SwerveModule.ZeroEncoders();
	*/

	FL_SwerveModule.SetTurnPID(0.9, 0, 1);
	FL_SwerveModule.SetDrivePIDF(0.4, 0, 0, 1);

	FR_SwerveModule.SetTurnPID(0.9, 0, 1);
	FR_SwerveModule.SetDrivePIDF(0.4, 0, 0, 1);

	BL_SwerveModule.SetTurnPID(0.9, 0, 1);
	BL_SwerveModule.SetDrivePIDF(0.4, 0, 0, 1);

	BR_SwerveModule.SetTurnPID(0.9, 0, 1);
	BR_SwerveModule.SetDrivePIDF(0.4, 0, 0, 1);


	a_HatchCollector.SetHatchPID(0.4, 0.00014, 0);
}

void Robot::RobotPeriodic(void)
{
	a_Gyro.Update();
}

void Robot::DisabledInit(void)
{
	robotState = "Disabled";
}

void Robot::DisabledPeriodic(void)
{

}

void Robot::TeleopInit(void)
{	
	// a_Gunnar.loop_start();
}

void Robot::TeleopPeriodic(void)
{
	robotState = "Teleoperated";

	// unsigned char rxBuf[8];
	// frc::CANData dataOne;
	// bool dataFound = a_Feather.ReadPacketNew(0, &dataOne);	
	a_Follower1.Update();


	if(a_Joystick1.GetRawButton(3)) // Zero Encoders
	{
		FL_SwerveModule.ZeroEncoders();
		FR_SwerveModule.ZeroEncoders();
		BL_SwerveModule.ZeroEncoders();
		BR_SwerveModule.ZeroEncoders();
	}

	

	if(a_Joystick1.GetRawButton(5))
	{
		if(targetAngle == -999)
		{
			targetAngle = a_Gyro.GetAngle(0);
			if(targetAngle < 0)
				targetAngle = 360 - ((int) (-1 * targetAngle) % 360); // Limits 
			else
				targetAngle = (int) targetAngle % 360;
		}
		float temp = 0;
		float dist = a_Follower1.GetPosInches();

		if((abs(dist) < 5) && ((-1.0 * dist > 0.33) || (dist > 0.33)))
		{
			temp = a_SwerveDrive.XForCenter(a_Follower1.GetPosInches());
		}

		frc::SmartDashboard::PutNumber("Virtual X Axis: ", temp);

		if(!(abs(a_Joystick1.GetRawAxis(1)) < DEADZONE))
		{
			// a_SwerveDrive.AngleLock(temp, -1 * a_Joystick1.GetRawAxis(1), targetAngle, a_Gyro.GetAngle(0), false);
		}
	}
	else if(a_Joystick1.GetRawButton(2))
	{
		// a_SwerveDrive.AngleLock(0, -1 * a_Joystick1.GetRawAxis(1), targetAngle, a_Gyro.GetAngle(0), false);
		if(targetAngle == -999)
		{
			targetAngle = a_Gyro.GetAngle(0);
			if(targetAngle < 0)
				targetAngle = 360 - ((int) (-1 * targetAngle) % 360); // Limits 
			else
				targetAngle = (int) targetAngle % 360;
		}
		a_SwerveDrive.AngleLock(0, -1 * a_Joystick1.GetRawAxis(1), targetAngle, a_Gyro.GetAngle(0), false);
	}
	else if(a_Joystick1.GetRawButton(1))
	{
		targetAngle = -999;
		// a_SwerveDrive.MakeshiftRotate(a_Joystick1.GetRawAxis(2) * 0.2);				
		a_SwerveDrive.SwerveDriveUpdate(-1 * a_Joystick1.GetRawAxis(0), -1 * a_Joystick1.GetRawAxis(1), -0.45 * a_Joystick1.GetRawAxis(2), a_Gyro.GetAngle(0));
		// a_SwerveDrive.AngleLock(-1 * a_Joystick1.GetRawAxis(0), -1 * a_Joystick1.GetRawAxis(1), 90, a_Gyro.GetAngle(0));
	}
	else // POSITIVE Z TURNS ROBOT TO THE LEFT
	{
		if(targetAngle == -999)
		{
			targetAngle = a_Gyro.GetAngle(0);
			if(targetAngle < 0)
				targetAngle = 360 - ((int) (-1 * targetAngle) % 360); // Limits 
			else
				targetAngle = (int) targetAngle % 360;
		}
		// a_SwerveDrive.SwerveDriveUpdate(-1 * a_Joystick1.GetRawAxis(0), -1 * a_Joystick1.GetRawAxis(1), 0, a_Gyro.GetAngle(0));
		// a_SwerveDrive.CrabGyro(-1 * a_Joystick1.GetRawAxis(0), -1 * a_Joystick1.GetRawAxis(1), a_Joystick1.GetRawAxis(2), a_Gyro.GetAngle(2));
		// a_SwerveDrive.CrabDrivePID(-1 * a_Joystick1.GetRawAxis(0), -1 * a_Joystick1.GetRawAxis(1), a_Joystick1.GetRawAxis(2));
		a_SwerveDrive.AngleLock(-1 * a_Joystick1.GetRawAxis(0), -1 * a_Joystick1.GetRawAxis(1), targetAngle, a_Gyro.GetAngle(0), true);
	}	


	if(a_Joystick1.GetRawButton(6))
	{
		a_Gyro.Cal();
	}


	if(a_Controller1.GetRawButton(3))
	{
		cargoDirection = false;
	}
	else if(a_Controller1.GetRawButton(4))
	{
		cargoDirection = true;
	}


	if(a_Controller1.GetRawButton(2)) // Annoying logic for a toggle for our collector.
	{
		if(!cargoLastInput)
		{
			if(!cargoToggle)
			{
				cargoToggle = true;
			}
			else
			{
				cargoToggle = false;
			}
		}
		cargoLastInput = true;
	}
	else
	{
		cargoLastInput = false;
	}

	if(cargoToggle)
	{
		a_CargoCollector.CargoCollectBB(a_Controller1.GetRawButton(1), cargoDirection);
	}
	else
	{
		a_CargoCollector.CargoAbort();
	}

	float crabbySpeed = 0.3;

	if(a_Controller1.GetRawAxis(3) == 0)
	{
		a_HatchCollector.UpdateRaw(crabbySpeed * a_Controller1.GetRawAxis(2)); 
	}
	else if(a_Controller1.GetRawAxis(2) == 0)
	{
		a_HatchCollector.UpdateRaw(-crabbySpeed * a_Controller1.GetRawAxis(3));	
	}
	else
	{
		a_HatchCollector.Disable();
	}
	
	if(a_Controller1.GetRawButton(6))
 	{
		if(a_HatchCollector.GetPositionRaw() > HATCH_POS_MID)
		{
			a_HatchCollector.SetHatchPID(0.5, 0.025, 3);
		}
		else
		{
			a_HatchCollector.SetHatchPID(0.4, 0.00014, 0);
		}
		a_HatchCollector.UpdateAngle((HATCH_POS_MAX) + 100);
 	}
	else if(a_Controller1.GetRawButton(5))
	{
		a_HatchCollector.SetHatchPID(0.4, 0.00014, 0);
		a_HatchCollector.UpdateAngle(HATCH_POS_MID);
	}
	
	/*
	if(a_Controller1.GetRawButton(5))
	{
		a_HatchCollector.UpdateAngle(HATCH_POS_MID);
	}
	
	if(a_Controller1.GetRawButton(6))
	{
		a_HatchCollector.UpdateAngle(HATCH_POS_MAX);
	}
	*/
	// hello -Jawad
	
	// a_HatchCollector.UpdateRaw(crabbySpeed * a_Controller1.GetRawAxis(1));

	// Stops Vision Data Loop
	if(a_Controller1.GetRawButton(10)) {
		// a_Gunnar.loop_stop();
	}

	float angleCounts = FR_SwerveModule.GetAngleRaw();
	float distanceCounts = FL_SwerveModule.GetDistanceRaw();
	float calibratedAngle = FR_SwerveModule.GetAngle();
	float distanceIn = FL_SwerveModule.GetDistanceIn();
	float distanceCm = FL_SwerveModule.GetDistanceCm();
	float currentOutput1 = FR_SwerveModule.GetCurrentOP(FR_DRIVE_ONE_ID);
	// currentOutput2 = FL_SwerveModule.GetCurrentOP(FL_DRIVE_TWO_ID);
	// currentOutput3 = FL_SwerveModule.GetCurrentOP(FL_TURN_ID);
	// voltageOutput1 = FL_SwerveModule.GetVoltageOP(FL_DRIVE_ONE_ID);
	// voltageOutput2 = FL_SwerveModule.GetVoltageOP(FL_DRIVE_TWO_ID);
	// voltageOutput3 = FL_SwerveModule.GetVoltageOP(FL_TURN_ID);


	// frc::SmartDashboard::PutNumber("FR Rotation: ", FR_SwerveModule.GetAngleRaw());
	// frc::SmartDashboard::PutNumber("FL Rotation: ", FL_SwerveModule.GetAngle());
	// frc::SmartDashboard::PutNumber("BR Rotation: ", BR_SwerveModule.GetAngleRaw());
	// frc::SmartDashboard::PutNumber("BL Rotation: ", BL_SwerveModule.GetAngleRaw());
	// frc::SmartDashboard::PutNumber("Distance Encoder: ", distanceCounts);
	frc::SmartDashboard::PutNumber("Calculated Angle: ", calibratedAngle);
	frc::SmartDashboard::PutNumber("Distance (In): ", distanceIn);
	frc::SmartDashboard::PutNumber("Distance (Cm): ", distanceCm);
	// frc::SmartDashboard::PutNumber("Drive Current 1: ", currentOutput1);
	// frc::SmartDashboard::PutNumber("Drive Current 2: ", currentOutput2);
	// frc::SmartDashboard::PutNumber("Turn Current: ", currentOutput3);
	// frc::SmartDashboard::PutNumber("Drive Voltage 1: ", voltageOutput1);
	// frc::SmartDashboard::PutNumber("Drive Voltage 2: ", voltageOutput2);
	// frc::SmartDashboard::PutNumber("Turn Voltage: ", voltageOutput3);
	// frc::SmartDashboard::PutBoolean("Cruise Control", cruiseControl);
	// frc::SmartDashboard::PutNumber("Gyro X:", a_Gyro.GetAngle(0));
	// frc::SmartDashboard::PutNumber("Gyro Y:", a_Gyro.GetAngle(1));
	// frc::SmartDashboard::PutNumber("Gyro Z:", a_Gyro.GetAngle(2)); 
	frc::SmartDashboard::PutNumber("Gyro Angle:", ((int)(a_Gyro.GetAngle(0) * 100))/100.0); // USE THIS ONE: Clockwise is negative
	// frc::SmartDashboard::PutNumber("BL Drive Encoder:", BL_SwerveModule.GetDistanceRaw());
	// frc::SmartDashboard::PutNumber("BR Drive Encoder:", BR_SwerveModule.GetDistanceRaw());
	// frc::SmartDashboard::PutNumber("FL Drive Encoder:", FL_SwerveModule.GetDistanceRaw());
	// frc::SmartDashboard::PutNumber("FR Drive Encoder:", FR_SwerveModule.GetDistanceRaw());
	frc::SmartDashboard::PutNumber("Hatch Encoder:", a_HatchCollector.GetPositionRaw());
	frc::SmartDashboard::PutBoolean("Beam Break?!?!??!?:", a_CargoCollector.GetCollectStatus());
	frc::SmartDashboard::PutBoolean("data?", a_Follower1.IsThereALine());
	frc::SmartDashboard::PutNumber("the data?", a_Follower1.GetPosInches());
	frc::SmartDashboard::PutNumber("Aarman but with two extra a's", a_Joystick1.GetPOV());
	// frc::SmartDashboard::PutNumber("Vision Distance:", a_Gunnar.GetDistance());
	// frc::SmartDashboard::PutNumber("Vision Angle:", a_Gunnar.GetAngle());
}

void Robot::AutonomousInit(void)
{
	robotState = "Autonomous";
	// a_Gunnar.loop_start();
}

void Robot::AutonomousPeriodic(void)
{

}

void Robot::TestInit(void)
{
	robotState = "Test";

}

void Robot::TestPeriodic(void)
{
	a_SwerveDrive.CrabGyro(-1 * a_Joystick1.GetRawAxis(0), -1 * a_Joystick1.GetRawAxis(1), a_Joystick1.GetRawAxis(2), a_Gyro.GetAngle(2));
	float angleCounts;
	float distanceCounts;
	float calibratedAngle;
	float distanceIn;
	float distanceCm;
	float currentOutput1;
	// float currentOutput2;
	// float currentOutput3;
	// float voltageOutput1;
	// float voltageOutput2;
	// float voltageOutput3;



	angleCounts = FR_SwerveModule.GetAngleRaw();
	distanceCounts = FL_SwerveModule.GetDistanceRaw();
	calibratedAngle = FR_SwerveModule.GetAngle();
	distanceIn = FL_SwerveModule.GetDistanceIn();
	distanceCm = FL_SwerveModule.GetDistanceCm();
	currentOutput1 = FR_SwerveModule.GetCurrentOP(FR_DRIVE_ONE_ID);
	// currentOutput2 = FL_SwerveModule.GetCurrentOP(FL_DRIVE_TWO_ID);
	// currentOutput3 = FL_SwerveModule.GetCurrentOP(FL_TURN_ID);
	// voltageOutput1 = FL_SwerveModule.GetVoltageOP(FL_DRIVE_ONE_ID);
	// voltageOutput2 = FL_SwerveModule.GetVoltageOP(FL_DRIVE_TWO_ID);
	// voltageOutput3 = FL_SwerveModule.GetVoltageOP(FL_TURN_ID);


	frc::SmartDashboard::PutNumber("Rotation Encoder: ", angleCounts);
	frc::SmartDashboard::PutNumber("Distance Encoder: ", distanceCounts);
	frc::SmartDashboard::PutNumber("Calculated Angle: ", calibratedAngle);
	frc::SmartDashboard::PutNumber("Distance (In): ", distanceIn);
	frc::SmartDashboard::PutNumber("Distance (Cm): ", distanceCm);
	frc::SmartDashboard::PutNumber("Drive Current 1: ", currentOutput1);
	// frc::SmartDashboard::PutNumber("Drive Current 2: ", currentOutput2);
	// frc::SmartDashboard::PutNumber("Turn Current: ", currentOutput3);
	// frc::SmartDashboard::PutNumber("Drive Voltage 1: ", voltageOutput1);
	// frc::SmartDashboard::PutNumber("Drive Voltage 2: ", voltageOutput2);
	// frc::SmartDashboard::PutNumber("Turn Voltage: ", voltageOutput3);
	frc::SmartDashboard::PutBoolean("Cruise Control", cruiseControl);
	frc::SmartDashboard::PutNumber("Gyro X:", a_Gyro.GetAngle(0));
	frc::SmartDashboard::PutNumber("Gyro Y:", a_Gyro.GetAngle(1));
	frc::SmartDashboard::PutNumber("Gyro Z:", a_Gyro.GetAngle(2)); // USE THIS ONE: Clockwise is negative
	frc::SmartDashboard::PutNumber("BL Drive Encoder:", BL_SwerveModule.GetDistanceRaw());
	frc::SmartDashboard::PutNumber("BR Drive Encoder:", BR_SwerveModule.GetDistanceRaw());
	frc::SmartDashboard::PutNumber("FL Drive Encoder:", FL_SwerveModule.GetDistanceRaw());
	frc::SmartDashboard::PutNumber("FR Drive Encoder:", FR_SwerveModule.GetDistanceRaw());
	frc::SmartDashboard::PutBoolean("Beam Break?!?!??!?:", a_CargoCollector.GetCollectStatus());
}


START_ROBOT_CLASS(Robot) // TODO: write our own main as this is deprecated
