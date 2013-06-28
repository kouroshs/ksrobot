#include <boost/python.hpp>

extern void ExportEigen();
extern void ExportPCL();
extern void ExportCommon();
extern void ExportInterfaces();

BOOST_PYTHON_MODULE(libksrobot)
{
    ExportEigen();
    ExportPCL();
    ExportCommon();
    ExportInterfaces();
}
