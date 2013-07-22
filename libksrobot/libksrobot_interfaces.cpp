#include <boost/python.hpp>

#include <interfaces/FovisInterface.h>
#include <interfaces/MTEngine.h>
#include <interfaces/KinectDatasetReader.h>
#include <interfaces/KinectDeviceReader.h>
#include <interfaces/OctomapInterface.h>

#include <boost/signals2.hpp>

class DummyScopeClass_Interfaces
{
};

void ExportInterfaces()
{
    using namespace boost::python;
    using namespace KSRobot::common;
    using namespace KSRobot::interfaces;
    
    scope namespace_scope = class_<DummyScopeClass_Interfaces>("interfaces");
    
    class_<FovisInterface, boost::shared_ptr<FovisInterface>, 
        bases<VisualOdometryInterface>, boost::noncopyable >("FovisInterface", init<>())
        .def("RegisterToKinect", &FovisInterface::RegisterToKinect)
        .def("Converged", &FovisInterface::Converged)
        .def("GetConvergenceError", &FovisInterface::GetConvergenceError)
        .def("RunSingleCycle", &FovisInterface::RunSingleCycle)
    ;
        
    class_<KinectDatasetReader, boost::shared_ptr<KinectDatasetReader>, 
        bases<KinectInterface>, boost::noncopyable >("KinectDatasetReader", init<>())
        .def("Initialize", &KinectDatasetReader::Initialize)
        .def("RunSingleCycle", &KinectDatasetReader::RunSingleCycle)
        .def("ProvidesGroundTruth", &KinectDatasetReader::ProvidesGroundTruth)
        .def("GetCurrentGroundTruth", &KinectDatasetReader::GetCurrentGroundTruth)
        .def("GetNumCycles", &KinectDatasetReader::GetNumCycles)
        .def("GetCurrentRgbFileName", &KinectDatasetReader::GetCurrentRgbFileName)
        .def("GetCurrentDepthFileName", &KinectDatasetReader::GetCurrentDepthFileName)
        .def("GetReadFiles", &KinectDatasetReader::GetReadFiles)
        .def("SetReadFiles", &KinectDatasetReader::SetReadFiles)
    ;

    class_<KinectDeviceReader, boost::shared_ptr<KinectDeviceReader>, 
        bases<KinectInterface>, boost::noncopyable >("KinectDeviceReader", init<>())
        .def("Initialize", &KinectDeviceReader::Initialize)
        .def("RunSingleCycle", &KinectDeviceReader::RunSingleCycle)
        .def("ProvidesGroundTruth", &KinectDeviceReader::ProvidesGroundTruth)
        .def("GetCurrentGroundTruth", &KinectDeviceReader::GetCurrentGroundTruth)
    ;
    
    class_<MTEngine, boost::shared_ptr<MTEngine>, 
        bases<EngineInterface>, boost::noncopyable >("MTEngine", init<>())
        .def("Initialize", &MTEngine::Initialize)
        .def("Start", &MTEngine::Start)
        .def("Stop", &MTEngine::Stop)
        .def("RunSingleCycle", &MTEngine::RunSingleCycle)
    ;
    
    class_<OctomapInterface, boost::shared_ptr<OctomapInterface>, 
        bases<MappingInterface>, boost::noncopyable >("OctomapInterface", no_init)
        .def("GetMapResolution", &OctomapInterface::GetMapResolution)
        .def("GetMaxRange", &OctomapInterface::GetMaxRange)
        .def("SetMapResolution", &OctomapInterface::SetMapResolution)
        .def("SetMaxRange", &OctomapInterface::SetMaxRange)
        .def("SetApplyVoxelGrid", &OctomapInterface::SetApplyVoxelGrid)
        .def("GetApplyVoxelGrid", &OctomapInterface::GetApplyVoxelGrid)
        .def("SetVoxelGridResolution", &OctomapInterface::SetVoxelGridResolution)
        .def("GetVoxelGridResolution", &OctomapInterface::GetVoxelGridResolution)
        .def("Initialize", &OctomapInterface::Initialize)
        .def("RegisterToVO", &OctomapInterface::RegisterToVO)
        .def("SaveToFile", &OctomapInterface::SaveToFile)
        .def("ReadSettings", &OctomapInterface::ReadSettings)
        .def("RunSingleCycle", &OctomapInterface::RunSingleCycle)
    ;
}
