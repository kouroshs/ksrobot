#ifndef KSROBOT_UTILS_DEFENITIONS_H
#define KSROBOT_UTILS_DEFENITIONS_H

#include <boost/chrono/duration.hpp>
#include <boost/chrono/time_point.hpp>
#include <boost/chrono/system_clocks.hpp>

// This is to define a function which exports the class
#define CLASS_DEF_PYEXPORT              public: static void ExportPython();
#define CLASS_PYEXPORT_FN(iface)        void iface::ExportPython()
#define REGISTER_CLASS(cls)             cls::ExportPython();

namespace KSRobot
{
    namespace common
    {
        typedef boost::chrono::high_resolution_clock    Clock;
        typedef Clock::time_point                       TimePoint;
        typedef Clock::duration                         Duration;
        
        inline long Milliseconds(const Duration& dur)
        {
            return boost::chrono::duration_cast<boost::chrono::milliseconds>(dur).count();
        }
        
        inline long Microseconds(const Duration& dur)
        {
            return boost::chrono::duration_cast<boost::chrono::microseconds>(dur).count();
        }
        
        inline long Nanoseconds(const Duration& dur)
        {
            return boost::chrono::duration_cast<boost::chrono::nanoseconds>(dur).count();
        }
        
        inline double Seconds(const Duration& dur)
        {
            return (double)Nanoseconds(dur) / 1.0e9;
        }
        
        void RegisterDebugModeStackTracePrinter();
        void PrintStackTrace();
    }
}

#endif //KSROBOT_UTILS_DEFENITIONS_H
