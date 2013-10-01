#include <interfaces/MTEngine.h>
#include <boost/python.hpp>

using namespace boost::python;
using namespace KSRobot::common;
using namespace KSRobot::interfaces;

CLASS_PYEXPORT_FN(MTEngine)
{
    class_<MTEngine, boost::shared_ptr<MTEngine>, 
        bases<EngineInterface>, boost::noncopyable >("MTEngine", init<>())
        .def("Initialize", &MTEngine::Initialize)
        .def("Start", &MTEngine::Start)
        .def("Stop", &MTEngine::Stop)
        .def("RunSingleCycle", &MTEngine::RunSingleCycle)
    ;
}
