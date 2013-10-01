#include <boost/python.hpp>
#include <common/Interface.h>
#include <common/ExecCtrlData.h>
#include <common/EngineInterface.h>
#include <common/KinectImage.h>
#include <common/KinectInterface.h>
#include <common/LoopDetectorInterface.h>
#include <common/MappingInterface.h>
#include <common/ProgramOptions.h>
#include <common/SLAMInterface.h>
#include <common/Timer.h>
#include <common/VisualOdometryInterface.h>
#include <common/RobotInfo.h>

using namespace boost::python;
using namespace KSRobot::common;

class DummyScopeClass_Common {;};

void ExportCommon()
{    
    scope namespace_scope = class_<DummyScopeClass_Common>("common");
    
    REGISTER_CLASS(Interface);
    REGISTER_CLASS(ExecCtrlData);
    REGISTER_CLASS(KinectImageDiskIO);
    REGISTER_CLASS(KinectInterface);
    REGISTER_CLASS(EngineInterface);
    REGISTER_CLASS(VisualOdometryInterface);
    REGISTER_CLASS(LoopDetectorInterface);
    REGISTER_CLASS(SLAMInterface);
    REGISTER_CLASS(MotionPlanner);
    REGISTER_CLASS(MappingInterface);
    REGISTER_CLASS(ProgramOptions);
    REGISTER_CLASS(OccupancyMap);
    REGISTER_CLASS(RobotInfo);
    REGISTER_CLASS(Timer);
}
