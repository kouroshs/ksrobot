#include <interfaces/RRTPlanner.h>
#include <boost/python.hpp>

using namespace boost::python;
using namespace KSRobot::common;
using namespace KSRobot::interfaces;

CLASS_PYEXPORT_FN(RRTPlanner)
{
    class_<RRTPlanner, RRTPlanner::Ptr, bases<MotionPlanner>, boost::noncopyable>("RRTPlanner", init<>())
        .def("Plan", &RRTPlanner::Plan)
    ;
}
