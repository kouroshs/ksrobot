#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <common/Interface.h>
#include <common/ExecCtrlData.h>
#include <common/EngineInterface.h>
#include <common/KinectImage.h>
#include <common/KinectInterface.h>
#include <common/LoopDetector.h>
#include <common/MappingInterface.h>
#include <common/ProgramOptions.h>
#include <common/SLAMInterface.h>
#include <common/Timer.h>
#include <common/VisualOdometryInterface.h>
#include <common/RobotInfo.h>

#include <libksrobot/interface_boost_optional.h>

using namespace boost::python;
using namespace KSRobot::common;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetIntOverloads, GetInt, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetDoubleOverloads, GetDouble, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetStringOverloads, GetString, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetBoolOverloads, GetBool, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(OccupancyMapExtractFrontiers, ExtractFrontiers, 1, 2)

template <class X>
struct DefineKinectImageType
{
    DefineKinectImageType<X>(const char* name)
    {
        using namespace boost::python;
        
        //TODO: Add more functions
        class_<X, typename X::Ptr>(name, init<>())
            .def(init<int, int>())
            .def("Create", &X::Create)
            .def("GetWidth", &X::GetWidth)
            .def("GetHeight", &X::GetHeight)
            .def("GetStride", &X::GetStride)
            .def("GetNumElements", &X::GetNumElements)
        ;
    }
};

#define DEF_KINECT_IMAGE_TYPE(type) DefineKinectImageType<type>(#type)

class DummyScopeClass_Common
{
};

struct STRUCT
{
    float x;
    float y;
    int rgba;
    
//     bool operator == (const STRUCT& o)
//     {
//         return true;
//     }
};

void OccupancyConvertToNumpy(KSRobot::common::OccupancyMap::Ptr ocmap)
{
    //TODO: IMPLEMENT THIS
}

void ExportCommon()
{    
    //Register boost::optional types to python
    KSRobot::pyksrobot::optional_<bool>();
    KSRobot::pyksrobot::optional_<int>();
    KSRobot::pyksrobot::optional_<double>();
    KSRobot::pyksrobot::optional_<std::string>();
    
    
    scope namespace_scope = class_<DummyScopeClass_Common>("common");
    
    class_<ProgramOptions, ProgramOptions::Ptr>("ProgramOptions", init<>())
        .def("LoadFromFile", &ProgramOptions::LoadFromFile)
        .def("SaveToFile", &ProgramOptions::SaveToFile)
        .def("NodeExists", &ProgramOptions::NodeExists)
        .def("StartNode", &ProgramOptions::StartNode)
        .def("GetInt", &ProgramOptions::GetInt, GetIntOverloads())
        .def("GetBool", &ProgramOptions::GetBool, GetBoolOverloads())
        .def("GetDouble", &ProgramOptions::GetDouble, GetDoubleOverloads())
        .def("GetString", &ProgramOptions::GetString, GetStringOverloads())
        .def("PutInt", &ProgramOptions::PutInt)
        .def("PutBool", &ProgramOptions::PutBool)
        .def("PutDouble", &ProgramOptions::PutDouble)
        .def("PutString", &ProgramOptions::PutString)
        ;
    
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
    ;
    
    DEF_KINECT_IMAGE_TYPE(KinectRgbImage);
    DEF_KINECT_IMAGE_TYPE(KinectRawDepthImage);
    DEF_KINECT_IMAGE_TYPE(KinectFloatDepthImage);
    
    class_<KinectImageDiskIO, boost::noncopyable>("KinectImageDiskIO", no_init)
        .def("SaveToFileRgb", &KinectImageDiskIO::SaveToFileRgb)
        .def("SaveToFileDepth", &KinectImageDiskIO::SaveToFileDepth)
        .def("LoadRgbFromFile", &KinectImageDiskIO::LoadRgbFromFile)
        .def("LoadDepthFromFile", &KinectImageDiskIO::LoadDepthFromFile)
        .staticmethod("SaveToFileRgb")
        .staticmethod("SaveToFileDepth")
        .staticmethod("LoadRgbFromFile")
        .staticmethod("LoadDepthFromFile")
    ;
    
    class_<pcl::PointXYZRGBA>("PointXYZRGBA", init<>())
        .def(init<const pcl::PointXYZRGBA&>())
        .def_readwrite("x", &pcl::PointXYZRGBA::x)
        .def_readwrite("y", &pcl::PointXYZRGBA::y)
        .def_readwrite("z", &pcl::PointXYZRGBA::z)
        .def_readwrite("r", &pcl::PointXYZRGBA::r)
        .def_readwrite("g", &pcl::PointXYZRGBA::g)
        .def_readwrite("b", &pcl::PointXYZRGBA::b)
        .def_readwrite("a", &pcl::PointXYZRGBA::a)
    ;
    
    class_<KinectPointCloud, KinectPointCloud::Ptr>("KinectPointCloud", init<>())
        .def(init<const KinectPointCloud&>())
        .def_readwrite("points", &KinectPointCloud::points)
        .def_readwrite("width", &KinectPointCloud::width)
        .def_readwrite("height", &KinectPointCloud::height)
        .def_readwrite("is_dense", &KinectPointCloud::is_dense)
        .def_readwrite("sensor_orientation_", &KinectPointCloud::sensor_orientation_)
        .def_readwrite("sensor_origin_", &KinectPointCloud::sensor_origin_)
        .def("size", &KinectPointCloud::size)
        .def("clear", &KinectPointCloud::clear)
    ;
    
    implicitly_convertible<KinectPointCloud::Ptr, KinectPointCloud::ConstPtr>();
    register_ptr_to_python<KinectPointCloud::ConstPtr>();
    
    {
        scope outer_ki =
        class_<KinectInterface, KinectInterface::Ptr, boost::noncopyable, bases<Interface> >("KinectInterface", no_init)
            .def("GetPointCloud", &KinectInterface::GetPointCloud)
            .def("GetRgbImage", &KinectInterface::GetRgbImage)
            .def("GetRawDepthImage", &KinectInterface::GetRawDepthImage)
            .def("GetFloatDepthImage", &KinectInterface::GetFloatDepthImage)
            .def("UnitsPerMeter", &KinectInterface::UnitsPerMeter)
            .def("GetCameraParams", &KinectInterface::GetCameraParams)
            .def("EnablePointCloudGeneration", &KinectInterface::EnablePointCloudGeneration)
            .def("EnableFloatDepthGeneration", &KinectInterface::EnableFloatDepthGeneration)
            .def("PointCloudGenerationEnabled", &KinectInterface::PointCloudGenerationEnabled)
            .def("FloatDepthGenerationEnabled", &KinectInterface::FloatDepthGenerationEnabled)
            .def("GeneratePointCloudFromImages", &KinectInterface::GeneratePointCloudFromImages)
            .def("Initialize", pure_virtual(&KinectInterface::Initialize))
            .def("ProvidesGroundTruth", pure_virtual(&KinectInterface::ProvidesGroundTruth))
            .def("GetCurrentGroundTruth", pure_virtual(&KinectInterface::GetCurrentGroundTruth))
            .staticmethod("GeneratePointCloudFromImages")
        ;
        
        class_<KinectInterface::CameraParameters>("CameraParameters")
            .def_readwrite("Width", &KinectInterface::CameraParameters::Width)
            .def_readwrite("Height", &KinectInterface::CameraParameters::Height)
            .def_readwrite("FocalX", &KinectInterface::CameraParameters::FocalX)
            .def_readwrite("FocalY", &KinectInterface::CameraParameters::FocalY)
            .def_readwrite("CenterX", &KinectInterface::CameraParameters::CenterX)
            .def_readwrite("CenterY", &KinectInterface::CameraParameters::CenterY)
            .def_readwrite("K1", &KinectInterface::CameraParameters::K1)
            .def_readwrite("K2", &KinectInterface::CameraParameters::K2)
            .def_readwrite("K3", &KinectInterface::CameraParameters::K3)
            .def_readwrite("P1", &KinectInterface::CameraParameters::P1)
            .def_readwrite("P2", &KinectInterface::CameraParameters::P2)
        ;
    }
    
    //TODO: complete this if necessary
    class_<ExecCtrlData>("ExecCtrlData", init<>())
        .def("CheckConsistancy", &ExecCtrlData::CheckConsistancy)
    ;
    
    class_<EngineInterface, EngineInterface::Ptr, boost::noncopyable, bases<Interface> >("EngineInterface", no_init)
        .def("SetExecutionParams", &EngineInterface::SetExecutionParams)
        .def("GetKinectInterface", &EngineInterface::GetKinectInterface)
        .def("GetVisualOdometryInterface", &EngineInterface::GetVisualOdometryInterface)
        .def("GetSlamInterface", &EngineInterface::GetSlamInterface)
        .def("GetMappingInterface", &EngineInterface::GetMappingInterface)
        .def("Initialize", &EngineInterface::Initialize)
    ;
    
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
    
    class_<MappingInterface, boost::shared_ptr<MappingInterface>, 
        bases<Interface>, boost::noncopyable>("MappingInterface", no_init)
        .def("SaveToFile", pure_virtual(&MappingInterface::SaveToFile))
        .def("LoadFromFile", pure_virtual(&MappingInterface::LoadFromFile))
        .def("ConvertToOccupancyGrid", pure_virtual(&MappingInterface::ConvertToOccupancyGrid))
        .def("SetRobotInfo", &MappingInterface::SetRobotInfo)
        .def("RegisterToVO", &MappingInterface::RegisterToVO)
    ;
    
    {
        class_<std::vector<OccupancyMap::MapPointDataType> >("OccupancyMapArray")
            .def(vector_indexing_suite<std::vector<OccupancyMap::MapPointDataType> >())
        ;
        
        scope occmap_class_scope =
        class_<OccupancyMap, boost::shared_ptr<OccupancyMap> >("OccupancyMap", init<>())
            .def(init<size_t, size_t>())
            .def("LoadFromFile", &OccupancyMap::LoadFromFile)
            .def("SaveToFile", &OccupancyMap::SaveToFile)
            .def("GetSize", &OccupancyMap::GetSize)
            .def("Index", &OccupancyMap::Index)
            .def("ConvertToNumpy", make_function(OccupancyConvertToNumpy))
            .def("IsValidIndex", &OccupancyMap::IsValidIndex)
            .def("IsValidPosition", &OccupancyMap::IsValidPosition)
            .def("AddPointToROI", &OccupancyMap::AddPointToROI)
            .def("ExtractFrontiers", &OccupancyMap::ExtractFrontiers, OccupancyMapExtractFrontiers())
            .def_readwrite("Width", &OccupancyMap::Width)
            .def_readwrite("Height", &OccupancyMap::Height)
            .def_readwrite("OccupiedCellsCount", &OccupancyMap::OccupiedCellsCount)
            .def_readwrite("FreeCellsCount", &OccupancyMap::FreeCellsCount)
            .def_readwrite("UnknownCellsCount", &OccupancyMap::UnknownCellsCount)
            .def_readwrite("CenterX", &OccupancyMap::CenterX)
            .def_readwrite("CenterY", &OccupancyMap::CenterY)
            .def_readwrite("Resolution", &OccupancyMap::Resolution)
            .def_readwrite("ROI", &OccupancyMap::ROI)
            .def_readwrite("Data", &OccupancyMap::Data)
        ;
        
        enum_<OccupancyMap::CellValue>("CellValue")
            .value("UnknownCell", OccupancyMap::UnknownCell)
            .value("OccupiedCell", OccupancyMap::OccupiedCell)
            .value("FreeCell", OccupancyMap::FreeCell)
            .value("FrontierCell", OccupancyMap::FrontierCell)
            .value("FrontierStartID", OccupancyMap::FrontierStartID)
            .value("FrontierEndID", OccupancyMap::FrontierEndID)
        ;
        
        class_<OccupancyMap::Rect>("Rect", init<>())
            .def_readwrite("Top", &OccupancyMap::Rect::Top)
            .def_readwrite("Left", &OccupancyMap::Rect::Left)
            .def_readwrite("Right", &OccupancyMap::Rect::Right)
            .def_readwrite("Buttom", &OccupancyMap::Rect::Buttom)
        ;
        
        //TODO: Needs definition of Eigen::Vector2f and Eigen::Matrix2f
        class_<OccupancyMap::Frontier>("Frontier", init<>())
            .def_readwrite("Position", &OccupancyMap::Frontier::Position)
            .def_readwrite("Variance", &OccupancyMap::Frontier::Variance)
            .def_readwrite("Rotation", &OccupancyMap::Frontier::Rotation)
            .def_readwrite("Size", &OccupancyMap::Frontier::Size)
            .def_readwrite("ID", &OccupancyMap::Frontier::ID)
        ;
        
        class_<std::vector<OccupancyMap::Frontier> >("FrontierArray")
            .def(vector_indexing_suite<std::vector<OccupancyMap::Frontier> >())
        ;
        
    }
    
    class_<RobotInfo, RobotInfo::Ptr>("RobotInfo", init<>())
        .def("ReadSettings", &RobotInfo::ReadSettings)
        .def("AddHeightToOdometry", &RobotInfo::AddHeightToOdometry)
        .def("ConstantHeight", &RobotInfo::GetConstantHeight)
        .def("Radius", &RobotInfo::GetRadius)
    ;
}