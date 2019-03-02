#pragma once

#include "frc/WPILib.h"
#include <frc/CAN.h>

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



#define NUM_SENSORS            32   // number of sensors used


class LineFollower
{
public:
	explicit LineFollower(int x_deviceID);
	virtual ~LineFollower();

	virtual void Update();
	virtual double GetPosInches();
    bool IsThereALine();


private:
    frc::CAN a_FeatherCAN;
    int deviceID;
    int sensorOutput[NUM_SENSORS];        // 1 or 0
    unsigned char rxBuf[8];


    // position of center of tape in inches
    double fpos = 0;

    // TOF sensor distance
    int tof_distance = 0;

    void decodeLineFollowerMsg();

};
