#include <interfaces/OctomapInterface.h>
#include <boost/python.hpp>

using namespace boost::python;
using namespace KSRobot::common;
using namespace KSRobot::interfaces;

CLASS_PYEXPORT_FN(OctomapInterface)
{
    class_<OctomapInterface, boost::shared_ptr<OctomapInterface>, 
        bases<MappingInterface>, boost::noncopyable >("OctomapInterface", init<>())
        .def("GetMapResolution", &OctomapInterface::GetMapResolution)
        .def("GetMaxRange", &OctomapInterface::GetMaxRange)
        .def("SetMapResolution", &OctomapInterface::SetMapResolution)
        .def("SetMaxRange", &OctomapInterface::SetMaxRange)
        .def("SetApplyVoxelGrid", &OctomapInterface::SetApplyVoxelGrid)
        .def("GetApplyVoxelGrid", &OctomapInterface::GetApplyVoxelGrid)
        .def("SetVoxelGridResolution", &OctomapInterface::SetVoxelGridResolution)
        .def("GetVoxelGridResolution", &OctomapInterface::GetVoxelGridResolution)
        .def("Initialize", &OctomapInterface::Initialize)
        .def("SaveToFile", &OctomapInterface::SaveToFile)
        .def("LoadFromFile", &OctomapInterface::LoadFromFile)
        .def("ReadSettings", &OctomapInterface::ReadSettings)
        .def("RunSingleCycle", &OctomapInterface::RunSingleCycle)
        .def("GetGroundPoints", &OctomapInterface::GetGroundPoints)
        .def("GetNonGroundPoints", &OctomapInterface::GetNonGroundPoints)
        .def("ConvertToOccupancyGrid", &OctomapInterface::ConvertToOccupancyGrid)
    ;
}
