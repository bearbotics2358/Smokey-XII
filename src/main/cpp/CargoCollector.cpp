#include <frc/WPILib.h>
#include <BeamBreak.h>
#include <CargoCollector.h>
#include <Prefs.h>


CargoCollector::CargoCollector()
: a_CargoMotor1(CARGO_SMALL_ID),
a_CargoMotor2(CARGO_BIG_ID),
a_BeamBreak()
{

}

void CargoCollector::CargoCollectBB(void){
if(a_BeamBreak.GetStatus() == false){
    
    CargoAbort();

}else if(a_BeamBreak.GetStatus()){
    CargoCollect();
}

}

void CargoCollector::CargoCollect(void){
a_CargoMotor1.Set(COLLECT_SPEED);
a_CargoMotor2.Set(COLLECT_SPEED);

}

void CargoCollector::CargoAbort(void) {

a_CargoMotor1.Set(0);
a_CargoMotor2.Set(0);

}