#include <WPILib.h>
#include <OSAL/Synchronized.h>
#include <OSAL/Task.h>

struct LogEntry
{
    uint32_t timestamp;
    uint32_t type;
    uint32_t channel;
    uint32_t value;
};

#define	LOG_INIT    0
#define	LOG_START   1
#define	LOG_STOP    2
#define LOG_MODE    3
#define LOG_CURRENT 4
#define LOG_SPEED   5
#define LOG_TACH    6

extern void LogInit( unsigned int size = 10000 );
extern void LogSave( const char *path );
extern void Log( uint32_t type, uint32_t channel, uint32_t value );

