#include <boost/python.hpp>
#include <interfaces/iSAM2Interface.h>

using namespace boost::python;
using namespace KSRobot::common;
using namespace KSRobot::interfaces;

CLASS_PYEXPORT_FN(iSAM2Interface)
{
    class_<iSAM2Interface, iSAM2Interface::Ptr, bases<SLAMInterface>, boost::noncopyable >("iSAM2Interface", init<>())
        .def("Initialize", &iSAM2Interface::Initialize)
        .def("ReadSettings", &iSAM2Interface::ReadSettings)
        .def("Print", &iSAM2Interface::Print)
    ;
}
