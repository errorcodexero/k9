#include <WPILib.h>
#include <OSAL/Synchronized.h>
#include <OSAL/Task.h>
#include "Tachometer.h"
#include "Logger.h"
#include <taskLib.h>

Tachometer::Tachometer( uint32_t channel ) :
    input(channel),
    lastTime(0),
    lastInterval(0),
    sampleValid(false),
    intervalValid(false)
{
    input.RequestInterrupts( Tachometer::InterruptHandler, this );
    input.EnableInterrupts();
}


Tachometer::~Tachometer()
{
    input.CancelInterrupts();
}


void
Tachometer::InterruptHandler( uint32_t mask, void *param )
{
    static_cast<Tachometer *>(param)->HandleInterrupt();
}


void
Tachometer::HandleInterrupt()
{
    // stupid floating point!
    uint32_t when = (uint32_t) (input.ReadInterruptTimestamp() * 1e6 + 0.5);
    {
	NTSynchronized LOCK(tachSem);

	if (sampleValid) {
	    uint32_t interval = when - lastTime;
	    if (interval < 200000) {	// 200mS
		lastInterval = interval;
		intervalValid = true;
	    }
	}
	lastTime = when;
	sampleValid = true;
    }

    Log(LOG_TACH, input.GetChannel(), when);
}


bool
Tachometer::GetInput()
{
    return input.Get();
}


uint32_t
Tachometer::GetInterval()
{
    NTSynchronized LOCK(tachSem);

    if (intervalValid) {
	// check if interval is _still_ valid
	uint32_t now = GetFPGATime();
	uint32_t interval = now - lastTime;
	if (interval < 200000) {	// 200mS
	    return lastInterval;
	} else {
	    intervalValid = false;
	}
    }
    return 0;
}


double
Tachometer::PIDGet()
{
    uint32_t interval = GetInterval();
    if (interval) {
	return 60.e6 / (double) interval;
    } else {
	return 0.;
    }
}

