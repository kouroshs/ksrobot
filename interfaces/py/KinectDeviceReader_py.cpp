#include <boost/python.hpp>
#include <interfaces/KinectDeviceReader.h>

using namespace boost::python;
using namespace KSRobot::common;
using namespace KSRobot::interfaces;

CLASS_PYEXPORT_FN(KinectDeviceReader)
{
    class_<KinectDeviceReader, boost::shared_ptr<KinectDeviceReader>, 
        bases<KinectInterface>, boost::noncopyable >("KinectDeviceReader", init<>())
        .def("Initialize", &KinectDeviceReader::Initialize)
        .def("RunSingleCycle", &KinectDeviceReader::RunSingleCycle)
        .def("ProvidesGroundTruth", &KinectDeviceReader::ProvidesGroundTruth)
        .def("GetCurrentGroundTruth", &KinectDeviceReader::GetCurrentGroundTruth)
    ;
}
