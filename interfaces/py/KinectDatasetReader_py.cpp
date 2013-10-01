#include <boost/python.hpp>
#include <interfaces/KinectDatasetReader.h>

using namespace boost::python;
using namespace KSRobot::common;
using namespace KSRobot::interfaces;

CLASS_PYEXPORT_FN(KinectDatasetReader)
{
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
}
