#include <common/RobotInfo.h>
#include <boost/python.hpp>

using namespace KSRobot::common;
using namespace boost::python;

CLASS_PYEXPORT_FN(RobotInfo)
{
    class_<RobotInfo, RobotInfo::Ptr>("RobotInfo", init<>())
        .def("ReadSettings", &RobotInfo::ReadSettings)
        .def("AddHeightToOdometry", &RobotInfo::AddHeightToOdometry)
        .def("ConstantHeight", &RobotInfo::GetConstantHeight)
        .def("Radius", &RobotInfo::GetRadius)
    ;
}
