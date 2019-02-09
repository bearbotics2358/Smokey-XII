#include <frc/WPILib.h>
#include "ctre\Phoenix.h"
#include <HatchCollector.h>
#include <Prefs.h>


HatchCollector::HatchCollector():
a_HatchMotor(HATCH_ID),
a_Potentiometer(0)

{

a_HatchMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::Analog, 0, 0);


}

void HatchCollector::UpdateRaw(int val) { 

if(GetPositionRaw() < HATCH_POS_MAX && GetPositionRaw() > HATCH_POS_MIN) {
    a_HatchMotor.Set(ControlMode::PercentOutput, val);
}
else
{
    Disable();
}

}
void HatchCollector::UpdateAngle(int angle){
    // will write after pid and testing  


}

float HatchCollector::GetPositionRaw(void){

float ret = a_Potentiometer.GetValue();
return ret;

}

float HatchCollector::GetPositionCalc(void){

float ret = Map(GetPositionRaw(), HATCH_POS_MIN, HATCH_POS_MAX, 90.0, 180.0); // numbers need to be tested with actual thing
return ret;

}

void HatchCollector::Disable(){
	a_HatchMotor.Set(0);
}



float HatchCollector::Map(float x, float in_min, float in_max, float out_min, float out_max){
    /*
        Adding some explanantion to this method since it may confuse some
        float x is the position
        in_max is the max value you can take in
        in_min is the min value you can take in
        out_max is the max value you want to be returned
        out_min is the min value you want to be returned
        returns a float between the desired bounds, based off position taken in and its bounds
        i.e. can turn raw into angle or angle into raw
    */
	float ret = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	return ret;
}