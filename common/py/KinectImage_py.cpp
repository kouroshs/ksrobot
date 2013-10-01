#include <common/KinectImage.h>
#include <boost/python.hpp>

using namespace KSRobot::common;
using namespace boost::python;

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


CLASS_PYEXPORT_FN(KinectImageDiskIO)
{
    //export all kinect images and everything

    DEF_KINECT_IMAGE_TYPE(KinectRgbImage);
    DEF_KINECT_IMAGE_TYPE(KinectRawDepthImage);
    DEF_KINECT_IMAGE_TYPE(KinectGrayImage);
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

}

