#include <frc/WPILib.h>
#include "LineFollower.h"
#include <HAL/HAL.h>
#include <LiveWindow/LiveWindow.h>



LineFollower::LineFollower(int x_deviceID):
    deviceID(x_deviceID),
    a_FeatherCAN(CAN_FOLLOWER | x_deviceID)
{
	// uint8_t Buff[256];
	// lastUpdate = 0;
	// Init();
	// printf("Reg 0 is: %d", GetReg0());
	// m_i2c = new I2C((I2C::Port)port, kAddress);
	// int ret = Read(0, 1, Buff);
	// printf("Jake Buff: %2.2X\n", Buff[0] & 0x00ff);

	// Specify the data format to read
	// SetRange(range);

	// HALReport(HALUsageReporting::kResourceType_ADXL345, HALUsageReporting::kJrimmyGyro, 0);
	// LiveWindow::GetInstance()->AddSensor("JrimmyGyro", port, this);
}

LineFollower::~LineFollower()
{
	// delete m_i2c;
	// m_i2c = NULL;
}

void LineFollower::Update()
{
// **********************************************************************************************
// Added for CAN Line Follower
// Should be moved to a new class, such as LineFollowerCAN

	// printf("in TestPeriodic\n");


	frc::CANData data1;

	bool ret = a_FeatherCAN.ReadPacketNew(0, &data1);
	
	int i;

	if(ret) {
		char stemp[9];

		for(i = 0; i < data1.length; i++) {
			rxBuf[i] = data1.data[i];
		}

		decodeLineFollowerMsg();
	}

// **********************************************************************************************
}

double LineFollower::GetPosInches()
{
  return fpos;

}

void LineFollower::decodeLineFollowerMsg()
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

bool LineFollower::IsThereALine()
{
  int i;
  int sum = 0;
  bool ret;

  for(i = 0; i < NUM_SENSORS; i++) {
    sum += sensorOutput[i];
  }

  ret = (sum > 0);

  return ret;
}