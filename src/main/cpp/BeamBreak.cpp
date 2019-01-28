#include <frc/WPILib.h>
#include <BeamBreak.h>
#include <Prefs.h>

BeamBreak::BeamBreak()
: a_Receiver(IR_RECEIVER_PORT) // REMEMBER TO ADD AN ID WHEN WE HAVE A BEAMBREAK
{
	inverted = false;
}

void BeamBreak::Init(){
	// InvertStatus();
}

void BeamBreak::InvertStatus(){
	inverted = true;
}

bool BeamBreak::GetStatus(){
	return (!((a_Receiver.Get() == 1) == inverted));
}
