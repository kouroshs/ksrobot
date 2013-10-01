#include <boost/python.hpp>
#include <interfaces/FovisInterface.h>

using namespace boost::python;
using namespace KSRobot::common;
using namespace KSRobot::interfaces;

CLASS_PYEXPORT_FN(FovisInterface)
{
    class_<FovisInterface, boost::shared_ptr<FovisInterface>, 
        bases<VisualOdometryInterface>, boost::noncopyable >("FovisInterface", init<>())
        .def("RegisterToKinect", &FovisInterface::RegisterToKinect)
        .def("Converged", &FovisInterface::Converged)
        .def("GetConvergenceError", &FovisInterface::GetConvergenceError)
        .def("RunSingleCycle", &FovisInterface::RunSingleCycle)
    ;
}