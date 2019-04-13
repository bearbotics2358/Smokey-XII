#ifndef SRC_BEAMBREAK_H_
#define SRC_BEAMBREAK_H_

#include <frc/WPILib.h>
#include <Prefs.h>

class BeamBreak
{
	public:
		BeamBreak();
		void Init();
		void InvertStatus();
		bool GetStatus();
	private:
		frc::DigitalInput a_Receiver;
		bool inverted;

};

#endif