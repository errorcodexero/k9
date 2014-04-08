#include <WPILib.h>
#include <OSAL/Synchronized.h>
#include <OSAL/Task.h>

// The Tachometer determines wheel speed by measuring the interval
// between rising edges of the Hall effect sensor output.

class Tachometer : public PIDSource
{
public:
    Tachometer( uint32_t channel );
    virtual ~Tachometer();

    uint32_t GetInterval( void );
    virtual double PIDGet( void );

private:
    DigitalInput sensor;
    NTReentrantSemaphore tachSem;

    uint32_t lastTime;
    uint32_t lastInterval;
    bool sampleValid;
    bool intervalValid;

    static void InterruptHandler( uint32_t mask, void *param );
    void HandleInterrupt( void );
};

