#include <common/ExecCtrlData.h>
#include <boost/python.hpp>

using namespace KSRobot::common;
using namespace boost::python;

CLASS_PYEXPORT_FN(ExecCtrlData)
{
    class_<ExecCtrlData>("ExecCtrlData", init<>())
        .def("CheckConsistancy", &ExecCtrlData::CheckConsistancy)
    ;
}
