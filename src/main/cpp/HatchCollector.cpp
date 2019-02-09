#include <frc/WPILib.h>
#include "ctre\Phoenix.h"
#include <HatchCollector.h>
#include <Prefs.h>


HatchCollector::HatchCollector():
a_HatchMotor1(HATCH_ID),
a_Potentiometer(0)

{



}

void HatchCollector::HatchCollect(int speed) {

if(a_Potentiometer.GetValue() < HATCH_POS_MAX && a_Potentiometer.GetValue() > HATCH_POS_MIN) {
a_HatchMotor1.Set(speed);
}

}









