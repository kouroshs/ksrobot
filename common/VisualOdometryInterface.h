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
#include <vector>

//TODO: Change keypoint to keyframe
//TODO: Implement features

namespace KSRobot
{
namespace common
{

class VisualFeature
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef boost::shared_ptr<VisualFeature>            Ptr;
    typedef boost::shared_ptr<const VisualFeature>      ConstPtr;
    
    Eigen::Vector3f         RelativePosition;
    float                   U, V;
    //TODO: Complete this!
    //TODO: ADD DESCRIPTOR, or use fovis descriptors. maybe add virtual functions?
};

class VisualOdometryInterface : public Interface
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef VisualOdometryInterface             this_type;
    typedef boost::shared_ptr<this_type>        Ptr;
    typedef boost::shared_ptr<const this_type>  ConstPtr;

    VisualOdometryInterface();
    virtual ~VisualOdometryInterface();
    
    struct Keyframe
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        Eigen::Isometry3f                                                       RelativeMotion;
        Eigen::Isometry3f                                                       GlobalPose;
        std::vector<VisualFeature, Eigen::aligned_allocator<VisualFeature> >    Features;
    };
    
    typedef std::vector<Keyframe, Eigen::aligned_allocator<Keyframe> >  KeyframeVector;
    
    virtual void                                RegisterToKinect(KinectInterface::Ptr ki);
    virtual bool                                Converged() = 0;
    virtual float                               GetConvergenceError() = 0;

    virtual void                                ReadSettings(ProgramOptions::Ptr po);
    virtual void                                WriteSettings(ProgramOptions::Ptr po);
    
    inline  bool                                IsThisCycleKeyframe() const;
    
    //NOTE: Important! must be called from children!
    virtual bool                                RunSingleCycle();
    
    inline KinectInterface::Ptr                 GetKinect() const;
    inline Eigen::Isometry3f                    GetMotionEstimate() const;
    inline Eigen::Isometry3f                    GetCurrRelativeMotion() const;
    inline const KeyframeVector&                GetKeyframes() const;
    inline const Keyframe&                      GetLastKeyframe() const;
    inline Keyframe&                            GetLastKeyframe();
    inline Eigen::Isometry3f                    GetGlobalPose() const;
    inline void                                 SetAxisTransform(const Eigen::Isometry3f& trans);
    inline Eigen::Isometry3f                    GetAxisTransform() const;
    
    // methods useful for when we are inside OnCycleFinished event.
    inline KinectPointCloud::ConstPtr           GetCurrentPointCloud() const;
    inline KinectRgbImage::ConstPtr             GetCurrentRgbImage() const;
    inline KinectRawDepthImage::ConstPtr        GetCurrentRawDepthImage() const;
    inline KinectFloatDepthImage::ConstPtr      GetCurrentFloatDepthImage() const;
    

    inline boost::signals2::connection          RegisterKeyframeReceiver(boost::function<void(const Keyframe&)> fn);
protected:
    void                                        ProjectToGround();
    void                                        NotifyKeyframeReceivers();
    // For VO, this function should be called after a motion estimate is calculated. 
    // It will update internal values such as global pose.
    virtual bool                                CheckForKeyframe();
    virtual void                                FinishCycle();
    virtual void                                AddKeyframeFeatures(Keyframe& kf) = 0;
protected:
    class MotionInfo
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        MotionInfo()
        {
            MotionEstimate.setIdentity();
            CurrRelativeMotion.setIdentity();
            GlobalPose.setIdentity();
            LastKeypointPose.setIdentity();
            AxisTransform.setIdentity();
        }
        
        Eigen::Isometry3f                       MotionEstimate;
        Eigen::Isometry3f                       CurrRelativeMotion;
        Eigen::Isometry3f                       GlobalPose;
        Eigen::Isometry3f                       LastKeypointPose;
        Eigen::Isometry3f                       AxisTransform;
    };
    
    MotionInfo*                                 mMotion;
    
    KinectInterface::Ptr                        mKinect;
    KeyframeVector                              mKeyframes;
    
    int                                         mLastKinectCycle;
    float                                       mMaxKeyframesDist;
    float                                       mMaxKeyframesAngle;
    float                                       mRobotHeight;
    bool                                        mIsCycleKeyframe;
    bool                                        mProjectOnGround;
    
    KinectPointCloud::ConstPtr                  mCurrPointCloud;
    KinectRgbImage::ConstPtr                    mCurrRgb;
    KinectRawDepthImage::ConstPtr               mCurrRawDepth;
    KinectFloatDepthImage::ConstPtr             mCurrFloatDepth;
    
    boost::signals2::signal<void(const Keyframe&)>             mKeyframeReceivers;
    Timer::Ptr                                  mOdomTimer;
};

inline Eigen::Isometry3f VisualOdometryInterface::GetMotionEstimate() const
{
    return mMotion->MotionEstimate;
}

inline Eigen::Isometry3f VisualOdometryInterface::GetCurrRelativeMotion() const
{
    return mMotion->CurrRelativeMotion;
}

inline const VisualOdometryInterface::KeyframeVector& VisualOdometryInterface::GetKeyframes() const
{
    return mKeyframes;
}

inline const VisualOdometryInterface::Keyframe& VisualOdometryInterface::GetLastKeyframe() const
{
    assert(mKeyframes.size() != 0);
    return mKeyframes.back();
}

inline VisualOdometryInterface::Keyframe& VisualOdometryInterface::GetLastKeyframe()
{
    assert(mKeyframes.size() != 0);
    return mKeyframes.back();
}


inline KinectInterface::Ptr VisualOdometryInterface::GetKinect() const
{
    return mKinect;
}

inline Eigen::Isometry3f VisualOdometryInterface::GetGlobalPose() const
{
    return mMotion->GlobalPose;
}

inline Eigen::Isometry3f VisualOdometryInterface::GetAxisTransform() const
{
    return mMotion->AxisTransform;
}

inline void VisualOdometryInterface::SetAxisTransform(const Eigen::Isometry3f& trans)
{
    assert(mMotion != NULL);
    mMotion->AxisTransform = trans;
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

inline boost::signals2::connection VisualOdometryInterface::RegisterKeyframeReceiver(boost::function<void(const Keyframe&)> fn)
{
    return mKeyframeReceivers.connect(fn);
}

inline bool VisualOdometryInterface::IsThisCycleKeyframe() const
{
    return mIsCycleKeyframe;
}

} // end namespace common
} // end namespace KSRobot

#endif // VISUALODOMETRYINTERFACE_H
