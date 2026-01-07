#ifndef STANDALONE_STUBS_HPP
#define STANDALONE_STUBS_HPP

#ifdef STANDALONE_HBA
#include <chrono>
#include <iostream>

namespace ros {
    namespace Time {
        struct StubTime {
            double toSec() const {
                auto now = std::chrono::high_resolution_clock::now();
                return std::chrono::duration<double>(now.time_since_epoch()).count();
            }
        };
        inline StubTime now() { return StubTime(); }
    }
    struct StubDuration {
        StubDuration(double d = 0) {}
    };
    typedef StubDuration Duration;
}

// Minimal ROS stubs for parameters
namespace ros {
    struct NodeHandle {
        NodeHandle(const std::string& s = "") {}
        template<typename T>
        bool getParam(const std::string& key, T& val) { return false; }
    };
    inline void init(int& argc, char** argv, const std::string& name) {}
}

#endif // STANDALONE_HBA

#endif // STANDALONE_STUBS_HPP

