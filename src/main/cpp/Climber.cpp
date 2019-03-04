#include <frc/WPILib.h>
#include "ctre\Phoenix.h"
#include <Climber.h>
#include <Prefs.h>


Climber::Climber():
a_VertiMotor(VERTI_ID)
// a_HoriMotor(HORI_ID)

{
    a_VertiMotor.ConfigSelectedFeedbackSensor(CTRE_MagEncoder_Relative, 0, 0);

   // a_HoriMotor.ConfigSelectedFeedbackSensor(CTRE_MagEncoder_Relative, 0, 0);

}

void Climber::Lift(float val){

a_VertiMotor.Set(val);

}

void Climber::Forward(float val){

// a_HoriMotor.Set(ControlMode::PercentOutput, val);

}

void Climber::LiftAuto(void){

a_VertiMotor.Set(ControlMode::Position, LIFTER_UP_POS);

}

void Climber::ForwardAuto(void){

// a_HoriMotor.Set(ControlMode::Position, LIFTER_FORWARD_POS);

}

void Climber::EndLiftAuto(void){

a_VertiMotor.Set(ControlMode::Position, LIFTER_FINAl_POS);

}

float Climber::GetPositionRaw(int motor){

float ret = 0;
if(motor == HORI_ID){
// ret = a_HoriMotor.GetSelectedSensorPosition(0);
}
else if(motor == VERTI_ID)
{
ret = a_VertiMotor.GetSelectedSensorPosition(0);
}


return ret;

}

float Climber::GetPositionCalc(void){
// Will do when we know conversion factors
// might add the map function here as well???????

}


        
void Climber::AutoAll(void){
    LiftAuto();
    ForwardAuto();
    EndLiftAuto();

    }