
#ifndef SRC_PREFS_H_
#define SRC_PREFS_H_


/********** CONTROLLER/JOYSTICK SETTINGS ***********/

#define JOYSTICK_PORT_ONE 1
#define CONTROLLER_PORT_ONE 2

//=========================//
#define DEADZONE 0.15
#define COLLECT_SPEED -0.65

/************** TALON CAN IDS *****************/

#define FL_DRIVE_ONE_ID 0
#define FL_TURN_ID 1

//=========================//

#define FR_DRIVE_ONE_ID 10 
#define FR_TURN_ID 11

//=========================//

#define BL_DRIVE_ONE_ID 20
#define BL_TURN_ID 21

//=========================//

#define BR_DRIVE_ONE_ID 30
#define BR_TURN_ID 31

//=========================//

#define CARGO_BACK_ID 2
#define CARGO_FRONT_ID 3

//=========================//

#define HATCH_ID 4

/********** PID CONTROLLER VALUES **********/

#define FL_TURN_P 0.9
#define FL_TURN_I 0
#define FL_TURN_D 1

#define FL_DRIVE_P 0.4
#define FL_DRIVE_F 1

//=========================//

#define FR_TURN_P 0.9
#define FR_TURN_I 0
#define FR_TURN_D 1

#define FR_DRIVE_P 0.4
#define FR_DRIVE_F 1

//=========================//

#define BL_TURN_P 0.9
#define BL_TURN_I 0
#define BL_TURN_D 1

#define BL_DRIVE_P 0.4
#define BL_DRIVE_F 1

//=========================//

#define BR_TURN_P 0.9
#define BR_TURN_I 0
#define BR_TURN_D 1

#define BR_DRIVE_P 0.4
#define BR_DRIVE_F 1

//=========================//

#define LOCK_P_GAIN 3.5
#define LOCK_I_GAIN 0.5
#define LOCK_D_GAIN 0.0
#define LOCK_F_GAIN 0.0

/********* VISION HOSTS AND PORTS *********/

#define bridge_host "10.23.58.26"

//=========================//

#define DEFAULT_PORT 1185
#define bridge_port DEFAULT_PORT

/************ POSITION ENCODER VALUES ************/

#define HATCH_POS_MAX 4650.0 
#define HATCH_POS_MID 4500.0
#define HATCH_POS_MIN 3310.0  

//=========================//

#define COUNTS_PER_ROTATION 12288 
#define COUNTS_PER_DRIVE_ROTATION -705

/******************* MEASUREMENTS AND RATIONS *******************/

#define GEAR_RATIO_SCALAR 2.8 // Gear Ratio from Drive Motor to Drive Wheel

//=========================//

#define WHEEL_CIRCUM_IN 9.4245
#define WHEEL_CIRCUM_CM 23.9390

//=========================//

#define DRIVE_TRAIN_SIDE_TO_SIDE 24.0
#define DRIVE_TRAIN_FRONT_TO_BACK 20.5

//=========================//

#define PI 3.14159265358979

/******************* MISC. PORTS AND ADDRESSES ********************/

#define LIGHT_RING_CONTROLLER_ADDRESS 0x4
#define IR_RECEIVER_PORT 1 

#endif
