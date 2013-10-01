#include <common/SLAMInterface.h>
#include <boost/python.hpp>

using namespace KSRobot::common;
using namespace boost::python;

CLASS_PYEXPORT_FN(SLAMInterface)
{
    class_<SLAMInterface, SLAMInterface::Ptr, bases<Interface>, boost::noncopyable>("SLAMInterface", no_init)
        .def("ReadFromFile", &SLAMInterface::ReadFromFile)
        .def("RegisterToVO", &SLAMInterface::RegisterToVO)
        .def("RegisterToLoopDetector", &SLAMInterface::RegisterToLoopDetector)
        .def("RunSingleCycle", &SLAMInterface::RunSingleCycle)
    ;
}
