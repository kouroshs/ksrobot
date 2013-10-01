#include <boost/python.hpp>

#include <interfaces/FovisInterface.h>
#include <interfaces/MTEngine.h>
#include <interfaces/KinectDatasetReader.h>
#include <interfaces/KinectDeviceReader.h>
#include <interfaces/OctomapInterface.h>
#include <interfaces/iSAM2Interface.h>
#include <interfaces/RRTPlanner.h>

using namespace boost::python;
using namespace KSRobot::common;
using namespace KSRobot::interfaces;

class DummyScopeClass_Interfaces {;};

void ExportInterfaces()
{
    scope namespace_scope = class_<DummyScopeClass_Interfaces>("interfaces");

    REGISTER_CLASS(FovisInterface);
    REGISTER_CLASS(KinectDeviceReader);
    REGISTER_CLASS(KinectDatasetReader);
    REGISTER_CLASS(MTEngine);
    REGISTER_CLASS(OctomapInterface);
    REGISTER_CLASS(iSAM2Interface);
    REGISTER_CLASS(RRTPlanner);
}
