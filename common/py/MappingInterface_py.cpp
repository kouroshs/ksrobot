#include <common/MappingInterface.h>
#include <boost/python.hpp>

using namespace KSRobot::common;
using namespace boost::python;

CLASS_PYEXPORT_FN(MappingInterface)
{
    class_<MappingInterface, boost::shared_ptr<MappingInterface>, 
        bases<Interface>, boost::noncopyable>("MappingInterface", no_init)
        .def("SaveToFile", pure_virtual(&MappingInterface::SaveToFile))
        .def("LoadFromFile", pure_virtual(&MappingInterface::LoadFromFile))
        .def("ConvertToOccupancyGrid", pure_virtual(&MappingInterface::ConvertToOccupancyGrid))
        .def("SetRobotInfo", &MappingInterface::SetRobotInfo)
        .def("RegisterToVO", &MappingInterface::RegisterToVO)
    ;
}