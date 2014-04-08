#include <WPILib.h>
#include <OSAL/Synchronized.h>
#include <OSAL/Task.h>
#include <vector>
#include <fstream>
#include "Logger.h"

static vector<LogEntry> *robotLog = NULL;
static NTReentrantSemaphore logSem;

void LogInit( unsigned int size )
{
cout << ">>> LogInit" << endl;
    NTSynchronized LOCK(logSem);

    if (!robotLog) {
	robotLog = new vector<LogEntry>();
	robotLog->reserve( size );
	Log(LOG_INIT, 0, 0);
    }
cout << "<<< LogInit" << endl;
}

void LogDump( const char *path )
{
cout << ">>> LogDump" << endl;
    NTSynchronized LOCK(logSem);

    if (robotLog && robotLog->size() > 1) {
	ofstream logFile(path, ofstream::out | ofstream::trunc);
	for (vector<LogEntry>::const_iterator it = robotLog->begin();
	     it != robotLog->end(); ++it)
	{
	    logFile << it->timestamp << ","
	    	    << it->type      << ","
		    << it->channel   << ","
		    << it->value     << endl;
	}
	robotLog->clear();
	Log(LOG_INIT, 0, 0);
    }
cout << "<<< LogDump" << endl;
}

void Log( uint32_t type, uint32_t channel, uint32_t value )
{
cout << ">>> Log(" << type << "," << channel << "," << value << ")" << endl;
    NTSynchronized LOCK(logSem);

    if (!robotLog) {
	LogInit();
    }

    LogEntry entry;
    entry.timestamp = GetFPGATime();
    entry.type = type;
    entry.channel = channel;
    entry.value = value;

    robotLog->push_back(entry);
cout << "<<< Log";
}


