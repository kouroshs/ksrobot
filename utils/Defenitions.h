#ifndef KSROBOT_UTILS_DEFENITIONS_H
#define KSROBOT_UTILS_DEFENITIONS_H

#include <boost/chrono.hpp>

namespace KSRobot
{
    namespace utils
    {
        typedef boost::chrono::high_resolution_clock    Clock;
        typedef Clock::time_point                       TimePoint;
        typedef Clock::duration                         Duration;
        
        long Milliseconds(const Duration& dur)
        {
            return boost::chrono::duration_cast<boost::chrono::milliseconds>(dur).count();
        }
        
    }
}

#endif //KSROBOT_UTILS_DEFENITIONS_H
