#ifndef SRC_CARGOCOLLECTOR_H_
#define SRC_CARGOCOLLECTOR_H_


#include <frc/WPILib.h>
#include "ctre\Phoenix.h"
#include <BeamBreak.h>

class CargoCollector 
{ 
public:
    CargoCollector(int front, int back);
    void CargoCollectBB(bool runOverride, bool dir);
    void CargoRun(bool dir);
    void CargoAbort(void);
    //void CargoKill(void);
    bool GetCollectStatus(void);

    
private:

WPI_TalonSRX a_CargoMotor1;
WPI_TalonSRX a_CargoMotor2;
BeamBreak a_BeamBreak;


};


#endif /* SRC_CARGOCOLLECTOR_H_ */
