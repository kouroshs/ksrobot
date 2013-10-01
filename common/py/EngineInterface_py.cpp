#include <common/EngineInterface.h>
#include <boost/python.hpp>

using namespace KSRobot::common;
using namespace boost::python;

CLASS_PYEXPORT_FN(EngineInterface)
{
    class_<EngineInterface, EngineInterface::Ptr, boost::noncopyable, bases<Interface> >("EngineInterface", no_init)
        .def("SetExecutionParams", &EngineInterface::SetExecutionParams)
        .def("GetKinectInterface", &EngineInterface::GetKinectInterface)
        .def("GetVisualOdometryInterface", &EngineInterface::GetVisualOdometryInterface)
        .def("GetSlamInterface", &EngineInterface::GetSlamInterface)
        .def("GetMappingInterface", &EngineInterface::GetMappingInterface)
        .def("Initialize", &EngineInterface::Initialize)
    ;
}