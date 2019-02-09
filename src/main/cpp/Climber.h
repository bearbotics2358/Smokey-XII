#ifndef SRC_Climber_H_
#define SRC_Climber_H_


#include <frc/WPILib.h>
#include "ctre\Phoenix.h"

class Climber 
{ 
public:
    Climber(void);
    void Lift(int val);     // ONLY FOR TESTING
    void Forward(int val);  // ONCE VALUES ARE FINALIZED THEN USE THE AUTO METHODS PLZ
    void LiftAuto(void);
    void ForwardAuto(void);
    void EndLiftAuto(void);
    void AutoAll(void);
    float GetPositionRaw(int motor);
    float GetPositionCalc(void);
    
private:

WPI_TalonSRX a_VertiMotor;
WPI_TalonSRX a_HoriMotor;


};


#endif /* SRC_Climber_H_ */
