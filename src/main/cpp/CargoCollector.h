#ifndef SRC_CARGOCOLLECTOR_H_
#define SRC_CARGOCOLLECTOR_H_


#include <WPILib.h>
#include "ctre\Phoenix.h"
#include <BeamBreak.h>

class CargoCollector 
{ 
public:
    CargoCollector(void);
    void CargoCollectBB(void);
    void CargoCollect(void);
    void CargoAbort(void);
    
private:

WPI_TalonSRX a_CollectMotor1;
WPI_TalonSRX a_CollectMotor2;
BeamBreak a_BeamBreak;
};


#endif /* SRC_CARGOCOLLECTOR_H_ */
