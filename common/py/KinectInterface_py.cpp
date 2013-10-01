#include <common/KinectInterface.h>
#include <boost/python.hpp>

using namespace KSRobot::common;
using namespace boost::python;

CLASS_PYEXPORT_FN(KinectInterface)
{
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

