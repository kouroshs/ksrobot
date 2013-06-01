#include "KinectInterface.h"

namespace KSRobot
{
namespace common
{


KinectInterface::KinectInterface(ProgramOptions::Ptr po) : mPO(po)
{
    //mConvertToGray = mPO->GetBool("ConvertToGray", false);
}

KinectInterface::~KinectInterface()
{

}

} // end namespace utils
} // end namespace KSRobot
