#ifndef SRC_HATCHCOLLECTOR_H_
#define SRC_HATCHCOLLECTOR_H_


#include <frc/WPILib.h>
#include "ctre\Phoenix.h"


class HatchCollector 
{ 
public:
    
    HatchCollector(void);
    void HatchCollect(int speed);


   
private:

WPI_TalonSRX a_HatchMotor1;

};


#endif /* SRC_HATCHCOLLECTOR_H_ */