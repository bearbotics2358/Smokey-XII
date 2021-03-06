#ifndef LIGHTRINGCONTROLLER_H
#define LIGHTRINGCONTROLLER_H

#include <frc/WPILib.h>

class LightRingController {
public:
	LightRingController();
	bool SetColor(int device, int r, int g, int b);
	void SetAllColor(int r, int g, int b);
	bool SetFlash(int device, bool set);
private:
	frc::I2C a_I2C;
};

#endif // LIGHTRINGCONTROLLER_H