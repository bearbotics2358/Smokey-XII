
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

#define CARGO_BIG_ID 2
#define CARGO_SMALL_ID 3

#define COLLECT_SPEED 0.5 // Will change with testing

#define COUNTS_PER_ROTATION 12288 // Encoder counts for one revolution
#define GEAR_RATIO_SCALAR 2.8 // Gear Ratio from Drive Motor to Drive Wheel

#define IR_RECEIVER_PORT 0 // Random #, we'll update it later.

#define WHEEL_CIRCUM_IN 9.4245
#define WHEEL_CIRCUM_CM 23.9390

#define DRIVE_TRAIN_SIDE_TO_SIDE 24.0
#define DRIVE_TRAIN_FRONT_TO_BACK 16.25

#define PI 3.14159265358979

#endif
