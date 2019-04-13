
#ifndef SRC_VIRTUALJOYSTICK_H
#define SRC_VIRTUALJOYSTICK_H

#include <PIDFController.h>

class VirtualJoystick 
{
    private:
        PIDFController PID;
    public:
        VirtualJoystick(float p, float i, float d, float f);
};

#endif