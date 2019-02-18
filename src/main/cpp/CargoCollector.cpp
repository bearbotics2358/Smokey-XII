#include <frc/WPILib.h>
#include <BeamBreak.h>
#include <CargoCollector.h>
#include <Prefs.h>


CargoCollector::CargoCollector():
a_CargoMotor1(CARGO_FRONT_ID),
a_CargoMotor2(CARGO_BACK_ID),
a_BeamBreak()

{

    a_CargoMotor1.ConfigSelectedFeedbackSensor(CTRE_MagEncoder_Absolute, 0, 0);
    a_CargoMotor1.Config_kP(0.5, 0, 0);
    a_CargoMotor1.Config_kI(0, 0, 0);
    a_CargoMotor1.Config_kD(0, 0, 0);
    a_CargoMotor1.Config_kF(0.22, 0, 0);

    a_CargoMotor2.ConfigSelectedFeedbackSensor(CTRE_MagEncoder_Absolute, 0, 0);
    a_CargoMotor2.Config_kP(0.5, 0, 0);
    a_CargoMotor2.Config_kI(0, 0, 0);
    a_CargoMotor2.Config_kD(0, 0, 0);
    a_CargoMotor2.Config_kF(0.22, 0, 0);
}

void CargoCollector::CargoCollectBB(bool runOverride, bool dir){ // IT'S NOT DUMB JASON >:U (feel better from software <3)

    if(runOverride)
    {
        CargoRun(dir);
    } 
    else    
    {
        if(a_BeamBreak.GetStatus())
        {
            CargoAbort();
        }
        else if(a_BeamBreak.GetStatus() == false)
        {
            CargoRun(dir);
        }
    }
}

void CargoCollector::CargoRun(bool dir)
{
   //  a_CargoMotor1.Set(ControlMode::Velocity, COLLECT_SPEED);
   //  a_CargoMotor2.Set(ControlMode::Velocity, COLLECT_SPEED);
    if(dir){
        a_CargoMotor1.Set(COLLECT_SPEED);
        a_CargoMotor2.Set(COLLECT_SPEED);
    }
    else
    {
        a_CargoMotor1.Set(-1 * COLLECT_SPEED);
        a_CargoMotor2.Set(-1 * COLLECT_SPEED);
    }
    

}
// :]
void CargoCollector::CargoAbort(void) 
{
    // a_CargoMotor1.Set(ControlMode::Velocity, 0);
    // a_CargoMotor2.Set(ControlMode::Velocity, 0);
    a_CargoMotor1.Set(0);
    a_CargoMotor2.Set(0);

}