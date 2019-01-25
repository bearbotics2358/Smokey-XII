#include <WPILib.h>
#include <BeamBreak.h>
#include <CargoCollector.h>
#include <Prefs.h>


CargoCollector::CargoCollector()
: a_CollectMotor1(CARGO_SMALL_ID),
a_CollectMotor2(CARGO_BIG_ID),
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
a_CollectMotor1.Set(COLLECT_SPEED);
a_CollectMotor2.Set(COLLECT_SPEED);

}

void CargoCollector::CargoAbort(void) {

a_CollectMotor1.Set(0);
a_CollectMotor2.Set(0);

}