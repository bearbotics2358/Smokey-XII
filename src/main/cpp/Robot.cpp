
#include <frc/WPILib.h> // <WPILib.h> is deprecated
#include <Prefs.h>
#include <Robot.h>
// o(╥﹏╥)o
// (>-.-)>-)====>
// <(0 w  0<)
//\(^u^)/
//^  ^
//  W
//   _________
//  / (0)  (0)\
// |      L    |
//  \____3____/
// ( * 3*) 
Robot::Robot(void):
a_Gyro(frc::I2C::kMXP),
a_Joystick1(JOYSTICK_PORT_ONE),
a_Controller1(CONTROLLER_PORT_ONE),
a_CargoCollector(CARGO_FRONT_ID, CARGO_BACK_ID),
a_HatchCollector(HATCH_ID),
FL_SwerveModule(FL_DRIVE_ONE_ID, FL_TURN_ID),
FR_SwerveModule(FR_DRIVE_ONE_ID, FR_TURN_ID),
BL_SwerveModule(BL_DRIVE_ONE_ID, BL_TURN_ID),
BR_SwerveModule(BR_DRIVE_ONE_ID, BR_TURN_ID),
a_SwerveDrive(),
// a_Climber(),
a_Interface(bridge_host, bridge_port),
// a_Gunnar("RIOclient", "localhost", 1183),
a_Follower1(1),
a_Light()
{
	// (>>'-')>> (Runs MQTT Broker on the RoboRio)
	const char *commandString = "/usr/local/sbin/mosquitto -p 1183 &"; // ampersand makes it run in the background!
	int q = system(commandString);
	printf("The number is: %d", q);
	cruiseControl = false;
	crabToggle = false;
	cargoToggle = false;
	cargoLastInput = false;
	cargoDirection = true;
	driveSpeed = 0;
	rotationSpeed = 0;
	targetAngle = -999;
	clawCamCirco = 1;
	cargoCamCirco = 0;
	counter = 0;
	frc::SmartDashboard::init();
}

void Robot::RobotInit(void)
{
	
	/*FL_SwerveModule.ZeroEncoders();
	FR_SwerveModule.ZeroEncoders();
	BL_SwerveModule.ZeroEncoders();
	BR_SwerveModule.ZeroEncoders();
	*/

	a_Interface.Init();
	a_Gyro.Init();
	a_Gyro.Cal();
	a_Gyro.Zero();

	a_Light.SetColor(3, 0, 50, 0);
	a_Light.SetColor(2, 0, 50, 0);
	a_Light.SetColor(1, 0, 50, 0);

	a_Interface.CargoOff();
	a_Interface.ClawViewing();

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
 	// a_Follower1.Update();
	a_Interface.Update();
	
}

void Robot::TeleopInit(void)
{	
	// a_Gunnar.loop_start();
}

void Robot::TeleopPeriodic(void)
{
	robotState = "Teleoperated";
	a_Interface.Update(); // a_vision.run(PNEUMATICS_FLUID);

	// unsigned char rxBuf[8];
	// frc::CANData dataOne;
	// bool dataFound = a_Feather.ReadPacketNew(0, &dataOne);	
	a_Follower1.Update();
	

	if(a_Joystick1.GetRawButton(5))
	{
		counter = 0;
		targetAngle = MBWOCFFHS(targetAngle);
		a_SwerveDrive.SwerveDriveUpdate(-1 * a_Joystick1.GetRawAxis(0), -1 * a_Joystick1.GetRawAxis(1), a_SwerveDrive.VisionZAxis(a_Interface.GetAngle()), a_Gyro.GetAngle(0));
		
	}
	else if(a_Joystick1.GetRawButton(2))
	{
		counter = 0;
		// a_SwerveDrive.AngleLock(0, -1 * a_Joystick1.GetRawAxis(1), targetAngle, a_Gyro.GetAngle(0), false);
		targetAngle = MBWOCFFHS(targetAngle);
		a_SwerveDrive.AngleLock(0, -1 * a_Joystick1.GetRawAxis(1), targetAngle, a_Gyro.GetAngle(0), false);
	}
	else if(a_Joystick1.GetRawButton(1))
	{
		counter = 0;
		targetAngle = -999;
		// a_SwerveDrive.MakeshiftRotate(a_Joystick1.GetRawAxis(2) * 0.2);
		if(a_Joystick1.GetRawButton(3))
		{
			a_SwerveDrive.SwerveRobotOriented(-1 * a_Joystick1.GetRawAxis(0), -1 * a_Joystick1.GetRawAxis(1), -0.6 * a_Joystick1.GetRawAxis(2));
		}
		else
		{
			a_SwerveDrive.SwerveDriveUpdate(-1 * a_Joystick1.GetRawAxis(0), -1 * a_Joystick1.GetRawAxis(1), -0.6 * a_Joystick1.GetRawAxis(2), a_Gyro.GetAngle(0));
		}
		// a_SwerveDrive.AngleLock(-1 * a_Joystick1.GetRawAxis(0), -1 * a_Joystick1.GetRawAxis(1), 90, a_Gyro.GetAngle(0));
	}
	/*else if(a_Joystick1.GetRawButton(3))
	{
		targetAngle = MBWOCFFHS(targetAngle);
		a_SwerveDrive.AngleLock(-1 * a_Joystick1.GetRawAxis(0), -1 * a_Joystick1.GetRawAxis(1), targetAngle, a_Gyro.GetAngle(0), false);
	}*/
	else // POSITIVE Z TURNS ROBOT TO THE LEFT
	{
		
		if(counter < 2)
		{
			a_SwerveDrive.SwerveDriveUpdate(-1 * a_Joystick1.GetRawAxis(0), -1 * a_Joystick1.GetRawAxis(1), 0, a_Gyro.GetAngle(0));
			counter++;
		} 
		else
		{
			targetAngle = MBWOCFFHS(targetAngle);
			// a_SwerveDrive.SwerveDriveUpdate(-1 * a_Joystick1.GetRawAxis(0), -1 * a_Joystick1.GetRawAxis(1), 0, a_Gyro.GetAngle(0));
			// a_SwerveDrive.CrabGyro(-1 * a_Joystick1.GetRawAxis(0), -1 * a_Joystick1.GetRawAxis(1), a_Joystick1.GetRawAxis(2), a_Gyro.GetAngle(2));
			// a_SwerveDrive.CrabDrivePID(-1 * a_Joystick1.GetRawAxis(0), -1 * a_Joystick1.GetRawAxis(1), a_Joystick1.GetRawAxis(2));
			a_SwerveDrive.AngleLock(-1 * a_Joystick1.GetRawAxis(0), -1 * a_Joystick1.GetRawAxis(1), targetAngle, a_Gyro.GetAngle(0), true);
		}

	}	


	if(a_Joystick1.GetRawButton(6))
	{
		a_Gyro.Cal();
		a_Gyro.Zero();
		targetAngle = 0;
	}
	

	if(a_Controller1.GetRawButton(3))
	{
		a_CargoCollector.CargoRun(false);
	
	}
	else if(a_Controller1.GetRawButton(4))
	{
		a_CargoCollector.CargoRun(true);
	}
	else
	{
		a_CargoCollector.CargoAbort();
	}	
		
	/* if(a_Controller1.GetRawButton(3))
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
	} */

	float crabbySpeed = 0.45;

	if(a_Controller1.GetRawButton(8))
	{
		crabbySpeed = 0.6;
	}
	else
	{
		crabbySpeed = 0.45;
	}

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
	

	float ClimberScalar = 0.85;

	if(a_Controller1.GetRawButton(7))
	{
		a_Climber.Lift(ClimberScalar * a_Controller1.GetRawAxis(1));
	}
	else
	{
		a_Climber.Lift(0);
	}



	if(a_Joystick1.GetRawButton(7))
	{
		if(clawCamCirco != 2)
		{
			clawCamCirco = 2;
			a_Interface.ClawViewing();
		}
	}
	if(a_Joystick1.GetRawButton(8))
	{
		if(cargoCamCirco != 2)
		{
			cargoCamCirco = 2;
			a_Interface.CargoViewing();
		}	
	}
	if(a_Joystick1.GetRawButton(9))
	{
		if(clawCamCirco != 1)
		{
			clawCamCirco = 1;
			a_Interface.ClawVision();
		}
	}
	if(a_Joystick1.GetRawButton(10))
	{
		if(cargoCamCirco != 1)
		{
			cargoCamCirco = 1;
			a_Interface.CargoVision();
		}	
	}
	if(a_Joystick1.GetRawButton(11))
	{
		if(clawCamCirco != 0)
		{
			clawCamCirco = 0;
			a_Interface.ClawOff();
		}
	}
	if(a_Joystick1.GetRawButton(12))
	{
		if(cargoCamCirco != 0)
		{
			cargoCamCirco = 0;
			a_Interface.CargoOff();
		}
		
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
	// hello - Jawad
	
	// a_HatchCollector.UpdateRaw(crabbySpeed * a_Controller1.GetRawAxis(1));



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


	frc::SmartDashboard::PutNumber("FR Rotation: ", FR_SwerveModule.GetAngle());
	frc::SmartDashboard::PutNumber("FL Rotation: ", FL_SwerveModule.GetAngle());
	frc::SmartDashboard::PutNumber("BR Rotation: ", BR_SwerveModule.GetAngle());
	frc::SmartDashboard::PutNumber("BL Rotation: ", BL_SwerveModule.GetAngle());
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
	frc::SmartDashboard::PutNumber("BL Drive Encoder:", BL_SwerveModule.GetDistanceRaw());
	frc::SmartDashboard::PutNumber("BR Drive Encoder:", BR_SwerveModule.GetDistanceRaw());
	frc::SmartDashboard::PutNumber("FL Drive Encoder:", FL_SwerveModule.GetDistanceRaw());
	frc::SmartDashboard::PutNumber("FR Drive Encoder:", FR_SwerveModule.GetDistanceRaw());
	/*frc::SmartDashboard::PutNumber("hatch camera", clawCamCirco);
	frc::SmartDashboard::PutNumber("cargo camera", cargoCamCirco);
	frc::SmartDashboard::PutNumber("Hatch Encoder:", a_HatchCollector.GetPositionRaw());
	frc::SmartDashboard::PutBoolean("Beam Break?!?!??!?:", a_CargoCollector.GetCollectStatus());
	frc::SmartDashboard::PutBoolean("data?", a_Follower1.IsThereALine());
	frc::SmartDashboard::PutNumber("the data?", a_Follower1.GetPosInches());
	frc::SmartDashboard::PutNumber("Aarman but with two extra a's", a_Joystick1.GetPOV());
	frc::SmartDashboard::PutNumber("Vision Distance:", a_Interface.GetDistance());
	frc::SmartDashboard::PutNumber("Vision Angle:", a_Interface.GetAngle());
	*/
}

void Robot::AutonomousInit(void)
{
	frc::SmartDashboard::PutNumber("Calibrated? ", 0);
	robotState = "Autonomous";
	//a_Gyro.Cal();
	a_Gyro.Zero();
	frc::SmartDashboard::PutNumber("Calibrated? ", 1);
	// a_Gunnar.loop_start();
}

void Robot::AutonomousPeriodic(void)
{
	TeleopPeriodic();
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

int Robot::MBWOCFFHS(int targetAng)
{
	if(targetAng == -999)
	{
		targetAng = a_Gyro.GetAngle(0);
		/*if(targetAng < 0)
			targetAng = 360 - ((int) (-1 * targetAng) % 360); // Limits 
		else
			targetAng = (int) targetAng % 360;*/
	}
	frc::SmartDashboard::PutNumber("Gyro Set Target: ",  targetAngle);
	return targetAng;
}

int main() { return frc::StartRobot<Robot>(); } // TODO: write our own main as this is deprecated
