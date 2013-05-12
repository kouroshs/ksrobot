#include "KinectInterface.h"

namespace KSRobot
{
namespace utils
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
