#include "crash_handler.h"
#include <csignal>
#include <cstdlib>
#include <execinfo.h>
#include <unistd.h>
#include <cxxabi.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <ctime>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/resource.h>
#include <sys/utsname.h>
#include <sys/syscall.h>
#include <cstring>
#include <ucontext.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <limits.h>
#include <vector>

namespace crash_handler {

static const int MAX_STACK_FRAMES = 128;
static const char* CRASH_LOG_DIR = "/tmp/fastlivo_crashes";
static volatile sig_atomic_t handler_active = 0;
static siginfo_t* g_siginfo = nullptr;
static ucontext_t* g_ucontext = nullptr;

std::string getCrashLogPath() {
    // Create directory if it doesn't exist
    struct stat info;
    if (stat(CRASH_LOG_DIR, &info) != 0) {
        mkdir(CRASH_LOG_DIR, 0755);
    }
    
    // Generate filename with timestamp
    std::time_t now = std::time(nullptr);
    char timestamp[64];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", std::localtime(&now));
    
    std::string filename = std::string(CRASH_LOG_DIR) + "/crash_" + timestamp + ".log";
    return filename;
}

void printSystemInfo(std::ostream& os) {
    struct utsname sys_info;
    if (uname(&sys_info) == 0) {
        os << "\n=== System Information ===" << std::endl;
        os << "OS: " << sys_info.sysname << " " << sys_info.release << std::endl;
        os << "Architecture: " << sys_info.machine << std::endl;
        os << "Hostname: " << sys_info.nodename << std::endl;
    }
}

void printMemoryInfo(std::ostream& os) {
    os << "\n=== Memory Information ===" << std::endl;
    
    // Read /proc/self/status for memory info
    std::ifstream status_file("/proc/self/status");
    if (status_file.is_open()) {
        std::string line;
        std::vector<std::string> mem_keys = {
            "VmPeak", "VmSize", "VmRSS", "VmData", "VmStk", "VmExe", "VmLib"
        };
        while (std::getline(status_file, line)) {
            for (const auto& key : mem_keys) {
                if (line.find(key) == 0) {
                    os << line << std::endl;
                }
            }
        }
        status_file.close();
    }
    
    // Stack limits
    struct rlimit rlim;
    if (getrlimit(RLIMIT_STACK, &rlim) == 0) {
        os << "Stack size limit: " << rlim.rlim_cur << " bytes (soft), " 
           << rlim.rlim_max << " bytes (hard)" << std::endl;
    }
}

void printSignalContext(std::ostream& os) {
    if (!g_siginfo) {
        return;
    }
    
    os << "\n=== Signal Context ===" << std::endl;
    os << "Signal code: " << g_siginfo->si_code << std::endl;
    
    if (g_siginfo->si_signo == SIGSEGV || g_siginfo->si_signo == SIGBUS) {
        os << "Fault address: " << std::hex << (void*)g_siginfo->si_addr << std::dec << std::endl;
        
        // Determine access type based on signal type
        if (g_siginfo->si_signo == SIGSEGV) {
            // SIGSEGV specific codes
            switch (g_siginfo->si_code) {
                case SEGV_MAPERR:
                    os << "Fault type: Address not mapped (null pointer or invalid address)" << std::endl;
                    break;
                case SEGV_ACCERR:
                    os << "Fault type: Invalid permissions (accessing read-only memory)" << std::endl;
                    break;
                default:
                    os << "Fault type: Unknown SIGSEGV code (" << g_siginfo->si_code << ")" << std::endl;
            }
        } else if (g_siginfo->si_signo == SIGBUS) {
            // SIGBUS specific codes
            switch (g_siginfo->si_code) {
                case BUS_ADRALN:
                    os << "Fault type: Invalid address alignment" << std::endl;
                    break;
                case BUS_ADRERR:
                    os << "Fault type: Non-existent physical address" << std::endl;
                    break;
                case BUS_OBJERR:
                    os << "Fault type: Object specific hardware error" << std::endl;
                    break;
                default:
                    os << "Fault type: Unknown SIGBUS code (" << g_siginfo->si_code << ")" << std::endl;
            }
        }
    }
}

void printRegisters(std::ostream& os) {
    if (!g_ucontext) {
        return;
    }
    
    os << "\n=== CPU Registers ===" << std::endl;
#ifdef __x86_64__
    mcontext_t* mctx = &g_ucontext->uc_mcontext;
    os << std::hex;
    os << "RIP: 0x" << mctx->gregs[REG_RIP] << std::endl;
    os << "RSP: 0x" << mctx->gregs[REG_RSP] << std::endl;
    os << "RBP: 0x" << mctx->gregs[REG_RBP] << std::endl;
    os << "RAX: 0x" << mctx->gregs[REG_RAX] << std::endl;
    os << "RBX: 0x" << mctx->gregs[REG_RBX] << std::endl;
    os << "RCX: 0x" << mctx->gregs[REG_RCX] << std::endl;
    os << "RDX: 0x" << mctx->gregs[REG_RDX] << std::endl;
    os << "RSI: 0x" << mctx->gregs[REG_RSI] << std::endl;
    os << "RDI: 0x" << mctx->gregs[REG_RDI] << std::endl;
    os << "R8:  0x" << mctx->gregs[REG_R8] << std::endl;
    os << "R9:  0x" << mctx->gregs[REG_R9] << std::endl;
    os << "R10: 0x" << mctx->gregs[REG_R10] << std::endl;
    os << "R11: 0x" << mctx->gregs[REG_R11] << std::endl;
    os << "R12: 0x" << mctx->gregs[REG_R12] << std::endl;
    os << "R13: 0x" << mctx->gregs[REG_R13] << std::endl;
    os << "R14: 0x" << mctx->gregs[REG_R14] << std::endl;
    os << "R15: 0x" << mctx->gregs[REG_R15] << std::endl;
    os << std::dec;
#elif defined(__aarch64__)
    mcontext_t* mctx = &g_ucontext->uc_mcontext;
    os << std::hex;
    os << "PC:  0x" << mctx->pc << std::endl;
    os << "SP:  0x" << mctx->sp << std::endl;
    if (mctx->regs) {
        os << "FP:  0x" << mctx->regs[29] << std::endl;
        os << "LR:  0x" << mctx->regs[30] << std::endl;
        for (int i = 0; i < 8; i++) {
            os << "X" << i << ":  0x" << mctx->regs[i] << std::endl;
        }
    }
    os << std::dec;
#else
    os << "Register information not available for this architecture" << std::endl;
#endif
}

void printMemoryMap(std::ostream& os) {
    os << "\n=== Memory Map (first 20 entries) ===" << std::endl;
    std::ifstream maps_file("/proc/self/maps");
    if (maps_file.is_open()) {
        std::string line;
        int count = 0;
        while (std::getline(maps_file, line) && count < 20) {
            os << line << std::endl;
            count++;
        }
        if (count >= 20) {
            os << "... (truncated, see /proc/self/maps for full map)" << std::endl;
        }
        maps_file.close();
    }
}

void printThreadInfo(std::ostream& os) {
    os << "\n=== Thread Information ===" << std::endl;
    os << "Main thread PID: " << getpid() << std::endl;
#ifdef SYS_gettid
    os << "Main thread TID: " << syscall(SYS_gettid) << std::endl;
#else
    os << "Main thread TID: " << getpid() << " (SYS_gettid not available)" << std::endl;
#endif
    
    // Count threads
    DIR* proc_dir = opendir("/proc/self/task");
    if (proc_dir) {
        int thread_count = 0;
        struct dirent* entry;
        while ((entry = readdir(proc_dir)) != nullptr) {
            if (entry->d_name[0] != '.') {
                thread_count++;
            }
        }
        closedir(proc_dir);
        os << "Total threads: " << thread_count << std::endl;
    }
}

void printEnvironmentInfo(std::ostream& os) {
    os << "\n=== Environment Variables ===" << std::endl;
    const char* env_vars[] = {
        "ROS_DOMAIN_ID", "ROS_DISTRO", "LD_LIBRARY_PATH", 
        "PATH", "HOME", "USER", "PWD"
    };
    for (const char* var : env_vars) {
        const char* value = getenv(var);
        if (value) {
            os << var << "=" << value << std::endl;
        }
    }
}

void printFileDescriptors(std::ostream& os) {
    os << "\n=== File Descriptors ===" << std::endl;
    DIR* fd_dir = opendir("/proc/self/fd");
    if (fd_dir) {
        int fd_count = 0;
        struct dirent* entry;
        while ((entry = readdir(fd_dir)) != nullptr) {
            if (entry->d_name[0] != '.') {
                fd_count++;
            }
        }
        closedir(fd_dir);
        os << "Open file descriptors: " << fd_count << std::endl;
    }
}

void printStackTrace(std::ostream& os, const char* executable_path = nullptr) {
    void* array[MAX_STACK_FRAMES];
    size_t size = backtrace(array, MAX_STACK_FRAMES);
    char** strings = backtrace_symbols(array, size);
    
    os << "\n=== Stack Trace ===" << std::endl;
    os << "Note: Use 'addr2line -e <executable> -f -C <address>' to get file:line info" << std::endl;
    if (executable_path) {
        os << "Executable: " << executable_path << std::endl;
    }
    os << std::endl;
    
    for (size_t i = 0; i < size; i++) {
        os << "[" << i << "] " << strings[i] << std::endl;
        
        // Try to demangle C++ symbols
        std::string symbol = strings[i];
        size_t start = symbol.find('(');
        size_t end = symbol.find('+', start);
        if (start != std::string::npos && end != std::string::npos) {
            std::string mangled = symbol.substr(start + 1, end - start - 1);
            int status = 0;
            char* demangled = abi::__cxa_demangle(mangled.c_str(), nullptr, nullptr, &status);
            if (status == 0 && demangled) {
                os << "     Demangled: " << demangled << std::endl;
                free(demangled);
            }
        }
        
        // Extract address for addr2line
        size_t addr_start = symbol.find('[');
        size_t addr_end = symbol.find(']', addr_start);
        if (addr_start != std::string::npos && addr_end != std::string::npos && executable_path) {
            std::string addr = symbol.substr(addr_start + 1, addr_end - addr_start - 1);
            os << "     addr2line: addr2line -e " << executable_path 
               << " -f -C " << addr << std::endl;
        }
    }
    free(strings);
}

void crashHandler(int sig, siginfo_t* siginfo, void* ucontext) {
    // Prevent recursive calls
    if (handler_active) {
        // Already handling a crash, restore default and re-raise
        signal(sig, SIG_DFL);
        raise(sig);
        return;
    }
    handler_active = 1;
    
    // Store context for detailed logging
    g_siginfo = siginfo;
    g_ucontext = (ucontext_t*)ucontext;
    
    // Get signal name
    const char* sig_name = "UNKNOWN";
    switch (sig) {
        case SIGSEGV: sig_name = "SIGSEGV (Segmentation Fault)"; break;
        case SIGABRT: sig_name = "SIGABRT (Abort)"; break;
        case SIGFPE:  sig_name = "SIGFPE (Floating Point Exception)"; break;
        case SIGILL:  sig_name = "SIGILL (Illegal Instruction)"; break;
        case SIGBUS:  sig_name = "SIGBUS (Bus Error)"; break;
        case SIGTERM: sig_name = "SIGTERM (Termination)"; break;
    }
    
    // Get executable path from /proc/self/exe
    char exe_path[1024] = {0};
    ssize_t len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
    if (len == -1) {
        exe_path[0] = '\0';
    } else {
        exe_path[len] = '\0';
    }
    
    // Get crash log path
    std::string log_path = getCrashLogPath();
    
    // Open log file
    std::ofstream log_file(log_path);
    std::ostringstream buffer;
    
    // Write crash information
    std::time_t now = std::time(nullptr);
    char time_str[64];
    std::strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
    
    buffer << "========================================" << std::endl;
    buffer << "FAST-LIVO2 CRASH REPORT" << std::endl;
    buffer << "========================================" << std::endl;
    buffer << "Time: " << time_str << std::endl;
    buffer << "Signal: " << sig_name << " (" << sig << ")" << std::endl;
    buffer << "PID: " << getpid() << std::endl;
    if (exe_path[0] != '\0') {
        buffer << "Executable: " << exe_path << std::endl;
    }
    buffer << "========================================" << std::endl;
    
    // Print detailed information
    printSystemInfo(buffer);
    printMemoryInfo(buffer);
    printThreadInfo(buffer);
    printFileDescriptors(buffer);
    printEnvironmentInfo(buffer);
    printSignalContext(buffer);
    printRegisters(buffer);
    printMemoryMap(buffer);
    
    // Print stack trace
    printStackTrace(buffer, exe_path[0] != '\0' ? exe_path : nullptr);
    
    buffer << "\n========================================" << std::endl;
    buffer << "Analysis Tips:" << std::endl;
    buffer << "1. Check the function where crash occurred (usually frame [3] or [4])" << std::endl;
    buffer << "2. Use addr2line to get exact file:line:" << std::endl;
    if (exe_path[0] != '\0') {
        buffer << "   addr2line -e " << exe_path << " -f -C <address>" << std::endl;
    }
    buffer << "3. Check for null pointer dereference, array out of bounds, or invalid memory access" << std::endl;
    buffer << "4. Common causes: uninitialized variables, use after free, stack overflow" << std::endl;
    buffer << "========================================" << std::endl;
    buffer << "End of crash report" << std::endl;
    buffer << "========================================" << std::endl;
    
    // Write to file
    log_file << buffer.str();
    log_file.close();
    
    // Also print to stderr
    std::cerr << "\n\n" << buffer.str() << std::endl;
    std::cerr << "\nCrash log saved to: " << log_path << std::endl;
    std::cerr.flush();
    
    // Restore default handler and re-raise signal
    signal(sig, SIG_DFL);
    raise(sig);
}

void installCrashHandlers() {
    if (handler_active) {
        return;
    }
    
    handler_active = 0;  // Initialize to 0, will be set to 1 when handling crash
    
    // Use sigaction instead of signal to get siginfo_t and ucontext_t
    struct sigaction sa;
    sa.sa_sigaction = crashHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_SIGINFO | SA_ONSTACK;  // SA_SIGINFO to get siginfo_t
    
    // Install handlers for common crash signals
    sigaction(SIGSEGV, &sa, nullptr);  // Segmentation fault
    sigaction(SIGABRT, &sa, nullptr);  // Abort
    sigaction(SIGFPE, &sa, nullptr);   // Floating point exception
    sigaction(SIGILL, &sa, nullptr);   // Illegal instruction
    sigaction(SIGBUS, &sa, nullptr);   // Bus error
    
    // Also handle termination to log graceful shutdowns
    sigaction(SIGTERM, &sa, nullptr);
}

} // namespace crash_handler

