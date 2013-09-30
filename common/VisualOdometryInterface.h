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
#include <common/RobotInfo.h>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <boost/signals2.hpp>
#include <vector>

#include <iostream>

namespace KSRobot
{
namespace common
{

class VisualFeature
{
public:
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;   // no need for this for Vector3f
    typedef boost::shared_ptr<VisualFeature>            Ptr;
    typedef boost::shared_ptr<const VisualFeature>      ConstPtr;
    
    Eigen::Vector3f         RelativePosition;
    float                   U, V;
    float                   Score;
};

class VisualFeatureData : public std::vector<unsigned char, Eigen::aligned_allocator<unsigned char> >
{
public:
    typedef boost::shared_ptr<VisualFeatureData>            Ptr;
    typedef boost::shared_ptr<const VisualFeatureData>      ConstPtr;
    
    size_t                              FeatureLength;
    size_t                              FeatureStride;
};

class VisualKeyframe
{
public:
    typedef boost::shared_ptr<VisualKeyframe>            Ptr;
    typedef boost::shared_ptr<const VisualKeyframe>      ConstPtr;

    struct MatchedPair
    {
        Eigen::Vector3f                                     CurrentRelPosition;
        Eigen::Vector3f                                     ReferenceRelPosition;
        int                                                 ReferenceIndex;
    };
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Eigen::Isometry3f                                                       RelativeMotion;
    Eigen::Isometry3f                                                       GlobalPose;
    std::vector<VisualFeature>                                              Features;
    VisualFeatureData                                                       DataPool;
    //TODO: Add feature matches to this class 
    
    size_t                                                                  CurrentCycle;
    size_t                                                                  ReferenceCycle;
    
    std::vector<MatchedPair>                                                MatchedPairs;
    bool                                                                    MotionEstimateAvailable;
    
    void Reset()
    {
        RelativeMotion.setIdentity();
        GlobalPose.setIdentity();
        Features.clear();
        DataPool.clear();
        MatchedPairs.clear();
        MotionEstimateAvailable = false;
    }
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
    
    virtual void                                RegisterToKinect(KinectInterface::Ptr ki);
    virtual bool                                Converged() = 0;
    virtual float                               GetConvergenceError() = 0;

    virtual void                                ReadSettings(ProgramOptions::Ptr po);
    virtual void                                SetRobotInfo(RobotInfo::Ptr roboinfo);
    
    inline  bool                                IsThisCycleKeyframe() const;
    
    //NOTE: Important! must be called from children!
    virtual bool                                RunSingleCycle();
    
    inline KinectInterface::Ptr                 GetKinect() const;
    inline Eigen::Isometry3f                    GetMotionEstimate() const;
    inline Eigen::Isometry3f                    GetCurrRelativeMotion() const;
    inline Eigen::Isometry3f                    GetGlobalPose() const;
    inline void                                 SetAxisTransform(const Eigen::Isometry3f& trans);
    inline Eigen::Isometry3f                    GetAxisTransform() const;
    inline VisualKeyframe::Ptr                  GetLatestKeyframe();
    
    // methods useful for when we are inside OnCycleFinished event.
    inline KinectPointCloud::ConstPtr           GetCurrentPointCloud() const;
    inline KinectRgbImage::ConstPtr             GetCurrentRgbImage() const;
    inline KinectGrayImage::ConstPtr            GetCurrentGrayImage() const;
    inline KinectRawDepthImage::ConstPtr        GetCurrentRawDepthImage() const;
    inline KinectFloatDepthImage::ConstPtr      GetCurrentFloatDepthImage() const;
    
    inline KinectPointCloud::ConstPtr           GetPreviousPointCloud() const;
    inline KinectRgbImage::ConstPtr             GetPreviousRgbImage() const;
    inline KinectGrayImage::ConstPtr            GetPreviousGrayImage() const;
    inline KinectRawDepthImage::ConstPtr        GetPreviousRawDepthImage() const;
    inline KinectFloatDepthImage::ConstPtr      GetPreviousFloatDepthImage() const;


    inline bool                                 KeyframeDescriptorPublishingEnabled() const;
    inline void                                 EnableKeyframeDescriptorPublishing(bool enable);

    inline boost::signals2::connection          RegisterKeyframeReceiver(boost::function<void(const VisualKeyframe::Ptr)> fn);
protected:
    void                                        NotifyKeyframeReceivers();
    // For VO, this function should be called after a motion estimate is calculated. 
    // It will update internal values such as global pose.
    virtual bool                                CheckForKeyframe();
    virtual void                                FinishCycle();
    virtual void                                PublishKeyframeFeatures(VisualKeyframe::Ptr kf) = 0;
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
            LastKeyframePose.setIdentity();
            AxisTransform.setIdentity();
        }
        
        Eigen::Isometry3f                       MotionEstimate;
        Eigen::Isometry3f                       CurrRelativeMotion;
        Eigen::Isometry3f                       GlobalPose;
        Eigen::Isometry3f                       LastKeyframePose;
        Eigen::Isometry3f                       AxisTransform;
    };
    
    
    
    MotionInfo*                                 mMotion;
    
    KinectInterface::Ptr                        mKinect;
    VisualKeyframe::Ptr                         mLatestKeyframe;
    
    int                                         mLastKinectCycle;
    bool                                        mIsCycleKeyframe;
    bool                                        mPublishKeyframeDescriptors;
    
    KinectPointCloud::ConstPtr                  mCurrPointCloud;
    KinectRgbImage::ConstPtr                    mCurrRgb;
    KinectGrayImage::ConstPtr                   mCurrGray;
    KinectRawDepthImage::ConstPtr               mCurrRawDepth;
    KinectFloatDepthImage::ConstPtr             mCurrFloatDepth;
    
    KinectPointCloud::ConstPtr                  mPrevPointCloud;
    KinectRgbImage::ConstPtr                    mPrevRgb;
    KinectGrayImage::ConstPtr                   mPrevGray;
    KinectRawDepthImage::ConstPtr               mPrevRawDepth;
    KinectFloatDepthImage::ConstPtr             mPrevFloatDepth;    
    
    boost::signals2::signal<void(const VisualKeyframe::Ptr)>             mKeyframeReceivers;
    Timer::Ptr                                  mOdomTimer;
    Timer::Ptr                                  mFinishCycleTime;
    
    bool                                        mSetHeight;
    float                                       mRobotHeight;
    
    bool                                        mIsEveryCycleKeyframe;
    bool                                        mUseMovementThresholdsForKeyframes;
    float                                       mMovementThr;
    float                                       mYawThr;
};

inline Eigen::Isometry3f VisualOdometryInterface::GetMotionEstimate() const
{
    return mMotion->MotionEstimate;
}

inline Eigen::Isometry3f VisualOdometryInterface::GetCurrRelativeMotion() const
{
    return mMotion->CurrRelativeMotion;
}

inline VisualKeyframe::Ptr VisualOdometryInterface::GetLatestKeyframe()
{
    return mLatestKeyframe;
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

inline KinectGrayImage::ConstPtr VisualOdometryInterface::GetCurrentGrayImage() const
{
    return mCurrGray;
}

inline KinectFloatDepthImage::ConstPtr VisualOdometryInterface::GetPreviousFloatDepthImage() const
{
    return mPrevFloatDepth;
}

inline KinectPointCloud::ConstPtr VisualOdometryInterface::GetPreviousPointCloud() const
{
    return mPrevPointCloud;
}

inline KinectRawDepthImage::ConstPtr VisualOdometryInterface::GetPreviousRawDepthImage() const
{
    return mPrevRawDepth;
}

inline KinectRgbImage::ConstPtr VisualOdometryInterface::GetPreviousRgbImage() const
{
    return mPrevRgb;
}

inline KinectGrayImage::ConstPtr VisualOdometryInterface::GetPreviousGrayImage() const
{
    return mCurrGray;
}

inline boost::signals2::connection VisualOdometryInterface::RegisterKeyframeReceiver(boost::function<void(const VisualKeyframe::Ptr)> fn)
{
    return mKeyframeReceivers.connect(fn);
}

inline bool VisualOdometryInterface::IsThisCycleKeyframe() const
{
    return mIsCycleKeyframe;
}

inline void VisualOdometryInterface::EnableKeyframeDescriptorPublishing(bool enable)
{
    mPublishKeyframeDescriptors = enable;
}

inline bool VisualOdometryInterface::KeyframeDescriptorPublishingEnabled() const
{
    return mPublishKeyframeDescriptors;
}


} // end namespace common
} // end namespace KSRobot

#endif // VISUALODOMETRYINTERFACE_H
