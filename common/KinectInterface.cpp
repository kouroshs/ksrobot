#include <common/KinectInterface.h>

namespace KSRobot
{
namespace common
{


KinectInterface::KinectInterface(ProgramOptions::Ptr po, const std::string& name) : Interface(po, name), mPC(new KinectPointCloud()),
    mRgb(new KinectRgbImage()), mRawDepth(new KinectRawDepthImage()), mFloatDepth(new KinectFloatDepthImage())
{
}

KinectInterface::~KinectInterface()
{
}

} // end namespace utils
} // end namespace KSRobot
