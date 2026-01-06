#ifndef CRASH_HANDLER_H
#define CRASH_HANDLER_H

#include <string>

namespace crash_handler {
    // Install signal handlers for common crash signals
    void installCrashHandlers();
    
    // Get crash log file path
    std::string getCrashLogPath();
}

#endif // CRASH_HANDLER_H




