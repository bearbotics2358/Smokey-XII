
#ifndef SRC_PREFS_H_
#define SRC_PREFS_H_

#define FL_DRIVE_ONE_ID 0// Temporary constant names, will add values once we choose which Talons we're using.
#define FL_TURN_ID 1

#define FR_DRIVE_ONE_ID 10 
#define FR_TURN_ID 11

#define BL_DRIVE_ONE_ID 20
#define BL_TURN_ID 21

#define BR_DRIVE_ONE_ID 30
#define BR_TURN_ID 31

#define JOYSTICK_PORT_ONE 1
#define CONTROLLER_PORT_ONE 2

#define CARGO_BACK_ID 2
#define CARGO_FRONT_ID 3
#define HATCH_ID 4
#define VERTI_ID 5
#define HORI_ID 6


// Vision
#define bridge_host "10.23.58.26"
#define DEFAULT_PORT 1185
#define bridge_port DEFAULT_PORT



#define COLLECT_SPEED -0.65
// Will change with testing
#define HATCH_POS_MAX 4650.0 // DO NOT FORGET TO CHANGE
#define HATCH_POS_MID 4500.0
#define HATCH_POS_MIN 3310.0   // GET FROM TESTING WITH POTENTIOMETER

#define LIFTER_UP_POS 1000     // ????
#define LIFTER_FORWARD_POS 100 // I've got no idea what these will be, need to be tested with the physical thing
#define LIFTER_FINAl_POS 800   // But these will be the positions the lifter goes to


#define COUNTS_PER_ROTATION 12288 // Encoder counts for one revolution
#define COUNTS_PER_DRIVE_ROTATION -705
#define GEAR_RATIO_SCALAR 2.8 // Gear Ratio from Drive Motor to Drive Wheel
#define DEADZONE 0.15


#define IR_RECEIVER_PORT 1 // Random #, we'll update it later.

#define WHEEL_CIRCUM_IN 9.4245
#define WHEEL_CIRCUM_CM 23.9390

#define DRIVE_TRAIN_SIDE_TO_SIDE 24.0
#define DRIVE_TRAIN_FRONT_TO_BACK 20.5

#define PI 3.14159265358979

#define LIGHT_RING_CONTROLLER_ADDRESS 0x4

// COMP CONSTANTS:
#define COMP_FL_TURN_PGAIN 0.9
#define COMP_BL_TURN_PGAIN 0.9
#define COMP_BR_TURN_PGAIN 0.9
#define COMP_FR_TURN_PGAIN 0.9

#define COMP_DGAIN 1

#endif
