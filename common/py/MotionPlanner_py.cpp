#include <common/MotionPlanner.h>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <vector>

using namespace KSRobot::common;
using namespace boost::python;

CLASS_PYEXPORT_FN(MotionPlanner)
{
    scope motion_planner_class_scope =
    class_<MotionPlanner, MotionPlanner::Ptr, boost::noncopyable>("MotionPlanner", no_init)
        .def("SetStartState", &MotionPlanner::SetStartState)
        .def("SetGoalState", &MotionPlanner::SetGoalState)
        .def("GetStartState", &MotionPlanner::GetStartState)
        .def("GetGoalState", &MotionPlanner::GetGoalState)
        .def("SetTimeout", &MotionPlanner::SetTimeout)
        .def("GetTimeout", &MotionPlanner::GetTimeout)
        .def("SetSafeThreshold", &MotionPlanner::SetSafeThreshold)
        .def("GetSafeThreshold", &MotionPlanner::GetSafeThreshold)
        .def("SetRobotRadius", &MotionPlanner::SetRobotRadius)
        .def("GetRobotRadius", &MotionPlanner::GetRobotRadius)
        .def("GetPlan", &MotionPlanner::GetPlan)
        .def("SetOccupancyMap", &MotionPlanner::SetOccupancyMap)
        .def("GetOccupancyMap", &MotionPlanner::GetOccupancyMap)
        
        .def("ReadSettings", &MotionPlanner::ReadSettings)
        .def("Initialize", &MotionPlanner::Initialize)
        
        .def("Plan", pure_virtual(&MotionPlanner::Plan))
    ;
    
    enum_<MotionPlanner::PlannerResult>("PlannerResult")
        .value("SUCCESS", MotionPlanner::SUCCESS)
        .value("TIMEOUT", MotionPlanner::TIMEOUT)
        .value("FAIL", MotionPlanner::FAIL)
    ;
    
    class_<MotionPlanner::RobotState>("RobotState", init<>())
        .def_readwrite("Position", &MotionPlanner::RobotState::Position)
        .def_readwrite("Yaw", &MotionPlanner::RobotState::Yaw)
    ;
    
    class_<std::vector<MotionPlanner::RobotState> >("StateVector")
        .def(vector_indexing_suite<std::vector<MotionPlanner::RobotState> >())
    ;
}
