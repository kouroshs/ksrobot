#include "Engine.h"

#include "kinect/KinectDatasetReader.h"
#include "kinect/KinectDeviceReader.h"

#include <boost/bind.hpp>
#include <boost/filesystem.hpp>

namespace KSRobot
{
namespace utils
{

namespace fs = boost::filesystem;

Engine::Engine(const ProgramOptions::Ptr po) : mPO(po), mKinectEnabled(true), mFovisEnabled(true), mOctoMapEnabled(true)
{
}

Engine::~Engine()
{
}

void Engine::Start()
{
    if( !GetKinectEnabled() )
        throw std::runtime_error("(Engine::Start) Kinect is not enabled.");

    //mKinectInterface->Start();
    //TODO: Change this
    if( mFovisEnabled )
        mKinectInterface->RegisterRGBDFloatCallback(boost::bind(&Engine::KinectRGBD, this, _1, _2));
    if( mOctoMapEnabled )
        mKinectInterface->RegisterPointCloudCallback(boost::bind(&Engine::KinectPC, this, _1));
}

void Engine::Stop()
{
}

void Engine::SetupKinect(const std::string& kinect)
{
    boost::filesystem::path p(kinect);
    if( fs::exists(p) )
        mKinectInterface.reset(new KinectDatasetReader(mPO->StartNode("KinectDatasetReader")));
    else
        mKinectInterface.reset(new KinectDeviceReader(mPO->StartNode("KinectDeviceReader")));

    mKinectInterface->Initialize(kinect);
}

bool Engine::GetKinectEnabled() const
{
    return mKinectEnabled;
}

void Engine::SetKinectEnabled(bool bEnable)
{
    mKinectEnabled = bEnable;
    if( mKinectEnabled == false )
        SetFovisEnabled(false);
}

bool Engine::GetFovisEnabled() const
{
    return mFovisEnabled;
}

void Engine::SetFovisEnabled(bool bEnable)
{
    mFovisEnabled = bEnable;
    if( mFovisEnabled == false )
        SetOctoMapEnabled(false);
}

bool Engine::GetOctoMapEnabled() const
{
    return mOctoMapEnabled;
}

void Engine::SetOctoMapEnabled(bool bEnable)
{
    mOctoMapEnabled = bEnable;
}

void Engine::KinectPC(const KinectPointCloud::ConstPtr& p)
{
//TODO: Complete this
}

void Engine::KinectRGBD(KinectRgbImage::Ptr rgb, KinectFloatDepthImage::Ptr depth)
{
//TODO: Complete this
}



} // end namespace utils
} // end namespace KSRobot
