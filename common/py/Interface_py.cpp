#include <common/Interface.h>
#include <boost/python.hpp>

using namespace boost::python;
using namespace KSRobot::common;

static void Interface_WriteRunningTimes(KSRobot::common::Interface::Ptr iface)
{
    iface->WriteRunningTimes(std::cout);
}

CLASS_PYEXPORT_FN(Interface)
{
    class_<Interface, Interface::Ptr, boost::noncopyable>("Interface", no_init)
        .def("GetName", &Interface::GetName)
        .def("GetCycle", &Interface::GetCycle)
        .def("LockData", &Interface::LockData)
        .def("UnlockData", &Interface::UnlockData)
        .def("SetHZ", &Interface::SetHZ)
        .def("SetPeriod", &Interface::SetPeriod)
        .def("Start", &Interface::Start)
        .def("Stop", &Interface::Stop)
        .def("ContinueExecution", &Interface::ContinueExecution)
        .def("RunSingleCycle", pure_virtual(&Interface::RunSingleCycle))
        .def("ReadSettings", &Interface::ReadSettings)
        .def("WriteSettings", &Interface::WriteSettings)
        .def("WriteRunningTimes", make_function(Interface_WriteRunningTimes))
    ;
}
