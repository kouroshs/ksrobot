#include <common/Timer.h>
#include <boost/python.hpp>

using namespace KSRobot::common;
using namespace boost::python;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(TimerToString, ToString, 0, 1)

CLASS_PYEXPORT_FN(Timer)
{
    class_<Timer, Timer::Ptr>("Timer", init<std::string>())
        .def("Start", &Timer::Start)
        .def("Stop", &Timer::Stop)
        .def("GetAverageTime", &Timer::GetAverageTime)
        .def("GetTimeVariance", &Timer::GetTimeVariance)
        .def("GetNumCalls", &Timer::GetNumCalls)
        .def("GetMaxTime", &Timer::GetMaxTime)
        .def("GetMinTime", &Timer::GetMinTime)
        .def("GetTotalTime", &Timer::GetTotalTime)
        .def("Name", &Timer::Name)
        .def("ToString", &Timer::ToString, TimerToString())
    ;
}
