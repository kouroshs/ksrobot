#include <common/LoopDetectorInterface.h>
#include <boost/python.hpp>

using namespace KSRobot::common;
using namespace boost::python;

CLASS_PYEXPORT_FN(LoopDetectorInterface)
{
    scope loop_detector_scope = 
    class_<LoopDetectorInterface, LoopDetectorInterface::Ptr, bases<Interface>, boost::noncopyable>("LoopDetectorInterface", no_init)
        .def("RegisterToVO", &LoopDetectorInterface::RegisterToVO)
        .def("RunSingleCycle", &LoopDetectorInterface::RunSingleCycle)
        .def("ReadSettings", &LoopDetectorInterface::ReadSettings)
    ;
        
    class_<LoopDetectorInterface::LoopClosure>("LoopClosure", init<>())
        .def_readwrite("Transform", &LoopDetectorInterface::LoopClosure::Transform)
        .def_readwrite("Cycle1", &LoopDetectorInterface::LoopClosure::Cycle1)
        .def_readwrite("Cycle2", &LoopDetectorInterface::LoopClosure::Cycle2)
    ;
}