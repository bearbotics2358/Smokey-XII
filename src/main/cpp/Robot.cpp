#include <frc/WPILib.h> // <WPILib.h> is deprecated
#include <Prefs.h>
#include <Robot.h>
#include <BeamBreak.h>


#define CAN_TEST

// (>-.-)>-)====>
Robot::Robot(void): 

#ifndef CAN_TEST

a_Gyro(frc::I2C::kMXP),
a_Joystick1(JOYSTICK_PORT_ONE),
FL_SwerveModule(FL_DRIVE_ONE_ID, FL_TURN_ID),
FR_SwerveModule(FR_DRIVE_ONE_ID, FR_TURN_ID),
BL_SwerveModule(BL_DRIVE_ONE_ID, BL_TURN_ID),
BR_SwerveModule(BR_DRIVE_ONE_ID, BR_TURN_ID),
a_BeamBreak(),
a_SwerveDrive(),

#endif

a_FeatherOne(1)

{
	// (>>'-')>>	
	
#ifndef CAN_TEST
	
	a_Gyro.Init();
	cruiseControl = false;
	crabToggle = false;
	driveSpeed = 0;
	rotationSpeed = 0;
	frc::SmartDashboard::init();

#endif

}

void Robot::RobotInit(void)
{

#ifndef CAN_TEST

	FL_SwerveModule.ZeroEncoders();
	FR_SwerveModule.ZeroEncoders();
	BL_SwerveModule.ZeroEncoders();
	BR_SwerveModule.ZeroEncoders();

	FL_SwerveModule.SetTurnPID(0.9, 0, 9);
	FL_SwerveModule.SetDrivePID(0.9, 0, 9);

	FR_SwerveModule.SetTurnPID(0.9, 0, 9);
	FR_SwerveModule.SetDrivePID(0.9, 0, 9);

	BL_SwerveModule.SetTurnPID(0.9, 0, 9);
	BL_SwerveModule.SetDrivePID(0.9, 0, 9);

	BR_SwerveModule.SetTurnPID(0.9, 0, 9);
	BR_SwerveModule.SetDrivePID(0.9, 0, 9);

#endif

}

void Robot::RobotPeriodic(void)
{

#ifndef CAN_TEST

	a_Gyro.Update();

#endif

}

void Robot::DisabledInit(void)
{

#ifndef CAN_TEST

	robotState = "Disabled";

#endif

}

void Robot::DisabledPeriodic(void)
{

#ifndef CAN_TEST



#endif

}

void Robot::TeleopInit(void)
{

#ifndef CAN_TEST



#endif

}

void Robot::TeleopPeriodic(void)
{



	printf("in Teleop Periodic\n");
	frc::SmartDashboard::PutString("debug", "Hello World");

#ifndef CAN_TEST


	robotState = "Teleoperated";

	if(a_Joystick1.GetRawButton(2)) // Enable Cruise Control
	{
		// cruiseControl = true;
		driveSpeed = a_Joystick1.GetRawAxis(1);
		rotationSpeed = a_Joystick1.GetRawAxis(0);
	}

	if(a_Joystick1.GetRawButton(3)) // Disable Cruise Control
	{
		// cruiseControl = false;
	}

	if(a_Joystick1.GetRawButton(8))
	{
		crabToggle = true;
	}

	if(a_Joystick1.GetRawButton(9))
	{
		crabToggle = false;
	}

	if(cruiseControl)
	{
		// FL_SwerveModule.UpdateRaw(driveSpeed, rotationSpeed);
	}
	else
	{
		if(crabToggle)
		{
			if(a_Joystick1.GetRawButton(1))
			{
				a_SwerveDrive.MakeshiftRotate(a_Joystick1.GetRawAxis(2) * 0.2);
			}
			else
			{
				a_SwerveDrive.CrabDrivePID(-1 *a_Joystick1.GetRawAxis(0), -1 *a_Joystick1.GetRawAxis(1), a_Joystick1.GetRawAxis(2));
			}	
		}
		else
		{
			FL_SwerveModule.UpdateRaw(a_Joystick1.GetRawAxis(1), a_Joystick1.GetRawAxis(0));
		}
	}

	if(a_Joystick1.GetRawButton(11))
	{
		FL_SwerveModule.UpdateAnglePID(350);
	}
	else if(a_Joystick1.GetRawButton(10))
	{
		FL_SwerveModule.UpdateAnglePID(10);
	}
	else if(a_Joystick1.GetRawButton(7))
	{
		FL_SwerveModule.UpdateAngle(-137);
	}
	else if(a_Joystick1.GetRawButton(6))
	{
		a_Gyro.Cal();
	}

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
	frc::SmartDashboard::PutBoolean("Beam Break?!?!??!?:", a_BeamBreak.GetStatus());

#endif

}

void Robot::AutonomousInit(void)
{

#ifndef CAN_TEST

	robotState = "Autonomous";

#endif

}

void Robot::AutonomousPeriodic(void)
{

#ifndef CAN_TEST



#endif

}

void Robot::TestInit(void)
{

	robotState = "Test";

}

// **********************************************************************************************
// Added for CAN Line Follower
// Should be moved to a new class, such as LineFollowerCAN
//
// Variables and constants should be in the class .h file
// decodeLineFollowerMsg() and other code in class .cpp file


/*
Calculate CAN ID for Line Follower msg transmission
NOTE: Confirm CAN ID matches the notebook - if not, the one in the notebook is correct

The Manufacturer (8 bits)
The Device Type (5 bits)
An API ID (10 bits)
The Device ID (6 bits)

For Team created modules, FIRST says that 
Manufacturer - HAL_CAN_Man_kTeamUse = 8
Device Type - HAL_CAN_Dev_kMiscellaneous = 10
API ID is up to us
Device ID is unique to each module of a specific type (e.g., we can have more than 1 line follower)

CAN ID: (Mfr ID): 0000 1000  (Device Type): 01010  (API ID): 00 0000 0000 (Device ID):00 0001 
CAN ID: 0 0001 0000 1010   0000 0000   0000 0001 
which is: 0x010a0001
*/
#define CAN_ID 0x010a0001
// CAN MASK is 1 bits in all 29 bit positions, except for the Device ID
#define CAN_MASK 0x01ffffc0
// all CAN followers will have the following result when ANDed with the CAN_MASK
#define CAN_FOLLOWER 0x010a0000

unsigned char rxBuf[8];

#define NUM_SENSORS            32   // number of sensors used
int sensorOutput[NUM_SENSORS];        // 1 or 0

// position of center of tape in inches
float fpos = 0;

// TOF sensor distance
int tof_distance = 0;



void decodeLineFollowerMsg()
{
  int i;
  int j;
  int16_t i16;

  // decode sensorOutput[] from bytes 0-3
  for(i = 0; i < 4; i++)
  {
    // get 8 sensor values from each byte
    for(j = 0; j < 8; j++) {
      sensorOutput[i * 8 + j] = (rxBuf[i] >> (7 - j)) & 0x01;
    }
  }
  for(i = 0; i < NUM_SENSORS; i++) {
		printf(" %d ", sensorOutput[i]);
  }
  printf("\n");

  // decode position
  // first, get the value as sent
  i16 = (rxBuf[4] << 8) | rxBuf[5];
  printf("pos (as sent): %d\n", i16);

  fpos = (i16 * 8.0)/25.4; // convert to inches - 8mm per sensor
  printf("pos (inches): %6.2f", fpos);

  // decode time of flight distance
  tof_distance = (rxBuf[6] << 8) | rxBuf[7];
  printf("tof_distance: %d\n", tof_distance);
}



// **********************************************************************************************



void Robot::TestPeriodic(void)
{




// **********************************************************************************************
// Added for CAN Line Follower
// Should be moved to a new class, such as LineFollowerCAN

	// printf("in TestPeriodic\n");
	frc::SmartDashboard::PutString("debug", "Hello World!");


	frc::CANData data1;

	bool ret = a_FeatherOne.ReadPacketNew(0, &data1);
	
	int i;

	if(ret) {
		char stemp[9];

		for(i = 0; i < data1.length; i++) {
			rxBuf[i] = data1.data[i];
		}

		decodeLineFollowerMsg();
	}

// **********************************************************************************************


#ifndef CAN_TEST


	// a_SwerveDrive.CrabDrivePID(a_Joystick1.GetRawAxis(0), a_Joystick1.GetRawAxis(1), a_Joystick1.GetRawAxis(2));
	/*if(a_Joystick1.GetRawButton(6))
	{
		FL_SwerveModule.UpdateAnglePID(90);
	}
	else if(a_Joystick1.GetRawButton(7))
	{
		FL_SwerveModule.UpdateAnglePID(0);
	}
	else
	{
		FL_SwerveModule.UpdateAnglePID(270);
	}
	*/

	// double calibratedAngle = FR_SwerveModule.GetAngle() + 180;
	// frc::SmartDashboard::PutNumber("Calculated Angle: ", calibratedAngle);

#endif	

}


START_ROBOT_CLASS(Robot) // TODO: write our own main as this is deprecated
