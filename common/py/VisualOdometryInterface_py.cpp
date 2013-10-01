#include <common/VisualOdometryInterface.h>
#include <boost/python.hpp>

using namespace KSRobot::common;
using namespace boost::python;

CLASS_PYEXPORT_FN(VisualOdometryInterface)
{
    class_<VisualOdometryInterface, VisualOdometryInterface::Ptr, 
        boost::noncopyable, bases<Interface> >("VisualOdometryInterface", no_init)
        
        .def("RegisterToKinect", &VisualOdometryInterface::RegisterToKinect)
        .def("IsThisCycleKeyframe", &VisualOdometryInterface::IsThisCycleKeyframe)
        .def("GetKinect", &VisualOdometryInterface::GetKinect)
        .def("GetMotionEstimate", &VisualOdometryInterface::GetMotionEstimate)
        .def("GetCurrRelativeMotion", &VisualOdometryInterface::GetCurrRelativeMotion)
        .def("GetGlobalPose", &VisualOdometryInterface::GetGlobalPose)
        .def("GetCurrentPointCloud", &VisualOdometryInterface::GetCurrentPointCloud)
        .def("GetCurrentRgbImage", &VisualOdometryInterface::GetCurrentRgbImage)
        .def("GetCurrentRawDepthImage", &VisualOdometryInterface::GetCurrentRawDepthImage)
        .def("GetCurrentFloatDepthImage", &VisualOdometryInterface::GetCurrentFloatDepthImage)
        
        .def("ReadSettings", &VisualOdometryInterface::ReadSettings)
        .def("WriteSettings", &VisualOdometryInterface::WriteSettings)
        
        .def("GetConvergenceError", pure_virtual(&VisualOdometryInterface::GetConvergenceError))
        .def("Converged", pure_virtual(&VisualOdometryInterface::Converged))
        
        .def("SetRobotInfo", &VisualOdometryInterface::SetRobotInfo)
    ;
}
