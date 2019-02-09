#include <frc/WPILib.h>
#include <BeamBreak.h>
#include <CargoCollector.h>
#include <Prefs.h>


CargoCollector::CargoCollector():
a_CargoMotor1(CARGO_SMALL_ID),
a_CargoMotor2(CARGO_BIG_ID),
a_BeamBreak()

{

    a_CargoMotor1.ConfigSelectedFeedbackSensor(CTRE_MagEncoder_Relative, 0, 0);

    a_CargoMotor2.ConfigSelectedFeedbackSensor(CTRE_MagEncoder_Relative, 0, 0);
    
}

void CargoCollector::CargoCollectBB(int runoverride){ // yeah i know this is dumb butt itll work well

if(runoverride == 1){
      CargoCollect();
}else{
    if(a_BeamBreak.GetStatus()){
       CargoAbort();
    }else if(a_BeamBreak.GetStatus() == false){
      CargoCollect();
    }

}

}

void CargoCollector::CargoCollect(void){
a_CargoMotor1.Set(ControlMode::Velocity, COLLECT_SPEED);
a_CargoMotor2.Set(ControlMode::Velocity, COLLECT_SPEED);

}

void CargoCollector::CargoAbort(void) {
a_CargoMotor1.Set(ControlMode::Velocity, 0);
a_CargoMotor2.Set(ControlMode::Velocity, 0);

}