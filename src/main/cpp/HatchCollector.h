#ifndef SRC_HATCHCOLLECTOR_H_
#define SRC_HATCHCOLLECTOR_H_


#include <frc/WPILib.h>
#include "ctre\Phoenix.h"


class HatchCollector 
{ 
public:
    
    HatchCollector(int hatchyboi);
    void SetHatchPID(float p, float i, float d);
    void UpdateRaw(float val); 
    void UpdateAngle(int angle);
    float GetPositionRaw(void);
    float GetPositionCalc(void);
    void Disable(void);


   
private:

    // frc::AnalogInput a_Potentiometer;
    WPI_TalonSRX a_HatchMotor;

    float Map(float x, float in_min, float in_max, float out_min, float out_max); // thx alocksis, this very useful <3

};


#endif /* SRC_HATCHCOLLECTOR_H_ */