#ifndef ENGINE_H
#define ENGINE_H

#include "ProgramOptions.h"
#include "kinect/KinectInterface.h"
#include "Worker.h"

#include <boost/thread.hpp>

#include <vector>

namespace KSRobot
{
namespace utils
{

//TODO Add SLAM
class Engine
{
public:
    Engine(const ProgramOptions::Ptr po);
    ~Engine();
    
    void                                SetupKinect(const std::string& kinect);
    void                                Start();
    void                                Stop();
    //TODO: Implement Pause, Resume
    //TODO: Implement externat receivers.
    //TODO: Implement clone method for KinectBaseImage
    
    void                                SetKinectEnabled(bool bEnable);
    bool                                GetKinectEnabled() const;
    void                                SetFovisEnabled(bool bEnable);
    bool                                GetFovisEnabled() const;
    void                                SetOctoMapEnabled(bool bEnable);
    bool                                GetOctoMapEnabled() const;

    KinectInterface::Ptr                GetKinectInterface() { return mKinectInterface; }
    
    
private:
    void                                KinectRGBD(KinectRgbImage::Ptr rgb, KinectFloatDepthImage::Ptr depth);
    void                                KinectPC(const KinectPointCloud::ConstPtr& p);
private:
    ProgramOptions::Ptr                 mPO;
    
    //KinectDatasetReader                 mDatasetReader;
    //KinectDeviceReader                  mDeviceReader;
    
    KinectInterface::Ptr                mKinectInterface;
    
    bool                                mKinectEnabled;
    bool                                mFovisEnabled;
    bool                                mOctoMapEnabled;
    
    Worker                              mKinectWorker;
    Worker                              mFovisWorker;
    Worker                              mOctoMapWorker;
    //Worker                              mPlannerWorker;
};

} // end namespace utils
} // end namespace KSRobot

#endif // ENGINE_H
