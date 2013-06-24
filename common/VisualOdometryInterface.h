/*
 * <one line to give the library's name and an idea of what it does.>
 * Copyright (C) 2013  Kourosh <email>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#ifndef VISUALODOMETRYINTERFACE_H
#define VISUALODOMETRYINTERFACE_H

#include <common/Interface.h>
#include <common/KinectInterface.h>
#include <common/Timer.h>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <boost/signals2.hpp>

//TODO: Change keypoint to keyframe
//TODO: Implement features

namespace KSRobot
{
namespace common
{

class VisualFeature
{
public:
    typedef boost::shared_ptr<VisualFeature>            Ptr;
    typedef boost::shared_ptr<const VisualFeature>      ConstPtr;
    
    float       U, V;
    Eigen::Vector3d       RelativePosition;
    //TODO: Complete this!
};

class VisualOdometryInterface : public Interface
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef VisualOdometryInterface             this_type;
    typedef boost::shared_ptr<this_type>        Ptr;
    typedef boost::shared_ptr<const this_type>  ConstPtr;

    VisualOdometryInterface(const std::string& name);
    virtual ~VisualOdometryInterface();
    
    struct Keypoint
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        Eigen::Isometry3d               RelativeMotion;
        Eigen::Isometry3d               GlobalPose;
    };
    
    typedef std::vector<Keypoint, Eigen::aligned_allocator<Keypoint> >  KeypointVector;
    
    virtual void                                RegisterToKinect(KinectInterface::Ptr ki);
    virtual bool                                Converged() = 0;
    virtual float                               GetConvergenceError() = 0;

    virtual void                                ReadSettings(ProgramOptions::Ptr po);
    virtual void                                WriteSettings(ProgramOptions::Ptr po);
    
    inline  bool                                IsThisCycleKeyframe() const;
    
    //NOTE: Important! must be called from children!
    virtual bool                                RunSingleCycle();
    
    inline KinectInterface::Ptr                 GetKinect() const;
    inline Eigen::Isometry3d                    GetMotionEstimate() const;
    inline Eigen::Isometry3d                    GetCurrRelativeMotion() const;
    inline const KeypointVector&                GetKeypointRelativeMotion() const;
    inline Eigen::Isometry3d                    GetGlobalPose() const;
    
    // methods useful for when we are inside OnCycleFinished event.
    inline KinectPointCloud::ConstPtr           GetCurrentPointCloud() const;
    inline KinectRgbImage::ConstPtr             GetCurrentRgbImage() const;
    inline KinectRawDepthImage::ConstPtr        GetCurrentRawDepthImage() const;
    inline KinectFloatDepthImage::ConstPtr      GetCurrentFloatDepthImage() const;

    inline boost::signals2::connection          RegisterKeyframeReceiver(boost::function<void()> fn);
protected:
    void                                        ProjectToGround();
    void                                        NotifyKeyframeReceivers();
    // For VO, this function should be called after a motion estimate is calculated. 
    // It will update internal values such as global pose.
    virtual void                                CheckForKeyframe();
    virtual void                                FinishCycle();
protected:
    struct MotionInfo
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Isometry3d                       MotionEstimate;
        Eigen::Isometry3d                       CurrRelativeMotion;
        Eigen::Isometry3d                       GlobalPose;
        Eigen::Isometry3d                       LastKeypointPose;
    };
    
    MotionInfo*                                 mMotion;
    
    KinectInterface::Ptr                        mKinect;
//     Eigen::Isometry3d                           mMotionEstimate;
//     Eigen::Isometry3d                           mCurrRelativeMotion;
//     Eigen::Isometry3d                           mGlobalPose;
//     Eigen::Isometry3d                           mLastKeypointPose;
    KeypointVector                              mKeypoints;
    
    int                                         mLastKinectCycle;
    double                                      mMaxKeyframesDist;
    double                                      mMaxKeyframesAngle;
    double                                      mRobotHeight;
    bool                                        mIsCycleKeyframe;
    bool                                        mProjectOnGround;
    
    KinectPointCloud::ConstPtr                  mCurrPointCloud;
    KinectRgbImage::ConstPtr                    mCurrRgb;
    KinectRawDepthImage::ConstPtr               mCurrRawDepth;
    KinectFloatDepthImage::ConstPtr             mCurrFloatDepth;
    
    boost::signals2::signal<void()>             mKeypointReceivers;
    Timer::Ptr                                  mOdomTimer;
};

inline Eigen::Isometry3d VisualOdometryInterface::GetMotionEstimate() const
{
    //return mMotionEstimate;
    return mMotion->MotionEstimate;
}

inline Eigen::Isometry3d VisualOdometryInterface::GetCurrRelativeMotion() const
{
    //return mCurrRelativeMotion;
    return mMotion->CurrRelativeMotion;
}

inline const VisualOdometryInterface::KeypointVector& VisualOdometryInterface::GetKeypointRelativeMotion() const
{
    return mKeypoints;
}

inline KinectInterface::Ptr VisualOdometryInterface::GetKinect() const
{
    return mKinect;
}

inline Eigen::Isometry3d VisualOdometryInterface::GetGlobalPose() const
{
    //return mGlobalPose;
    return mMotion->GlobalPose;
}

inline KinectFloatDepthImage::ConstPtr VisualOdometryInterface::GetCurrentFloatDepthImage() const
{
    return mCurrFloatDepth;
}

inline KinectPointCloud::ConstPtr VisualOdometryInterface::GetCurrentPointCloud() const
{
    return mCurrPointCloud;
}

inline KinectRawDepthImage::ConstPtr VisualOdometryInterface::GetCurrentRawDepthImage() const
{
    return mCurrRawDepth;
}

inline KinectRgbImage::ConstPtr VisualOdometryInterface::GetCurrentRgbImage() const
{
    return mCurrRgb;
}

boost::signals2::connection VisualOdometryInterface::RegisterKeyframeReceiver(boost::function<void()> fn)
{
    return mKeypointReceivers.connect(fn);
}

inline bool VisualOdometryInterface::IsThisCycleKeyframe() const
{
    return mIsCycleKeyframe;
}

} // end namespace common
} // end namespace KSRobot

#endif // VISUALODOMETRYINTERFACE_H
