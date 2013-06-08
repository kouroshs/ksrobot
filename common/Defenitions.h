#ifndef KSROBOT_UTILS_DEFENITIONS_H
#define KSROBOT_UTILS_DEFENITIONS_H

#include <boost/chrono/duration.hpp>
#include <boost/chrono/time_point.hpp>
#include <boost/chrono/system_clocks.hpp>

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
        
    }
}

#endif //KSROBOT_UTILS_DEFENITIONS_H
