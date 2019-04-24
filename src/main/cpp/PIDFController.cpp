
#include <PIDFController.h>

PIDFController::PIDFController(float p, float i, float d, float f)
{
    pGain = p;
    iGain = i;
    dGain = d;
    fGain = f;

    error = 0;
    integral = 0;
    derivative = 0;

    cycle = 1;
}

PIDFController::PIDFController(float p, float i, float d)
{
    pGain = p;
    iGain = i;
    dGain = d;
    fGain = 0;

    error = 0;
    integral = 0;
    derivative = 0;

    cycle = 1;
}

PIDFController::~PIDFController()
{

}

float PIDFController::GetOutput(void)
{
    // Output = outputScalar * (P * error) + (I * integral) + (D * derivative)
    float outputScalar = 1.0;
    return outputScalar * ((pGain * error) + (iGain * integral) + (dGain * derivative)); // TODO: add f
}

float PIDFController::GetError(void)
{
    return error;
}

void PIDFController::Update(float target,float current) // Method to update error, integral, and derivative values
{
    float lastError = error;
    error = (current - target) / cycle;

    integral += (0.5) * (0.02) * (lastError + error); // Approximates integral of error via trapezoidal riemann sum
    // Integral = Area of Trapezoid = 1/2 * (h) + (b1 + b2) = 1/2 * (time in between cycles) * (error + lastError)
    // I assume that it has been 20 ms, or 0.02 of a second as it is 17.5 +/- 2.5 ms per cycle and I is slow anyway 

    derivative =  error - lastError; // Approximates derivative as a straight line between current and last cycle's error
}

float PIDFController::GetCycle(void)
{
    return cycle;
}

void PIDFController::SetCycle(float inCycle)
{
    cycle = inCycle;
}

void PIDFController::ResetError(void)
{
    error = 0;
    integral = 0;
    derivative = 0;
}