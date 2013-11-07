/*
 * <one line to give the library's name and an idea of what it does.>
 * Copyright (C) 2013  Kourosh <kourosh.sartipi@gmail.com>
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

#ifndef FOVISINTERFACE_H
#define FOVISINTERFACE_H

#include <common/VisualOdometryInterface.h>
#include <common/Timer.h>
#include <common/PCLUtils.h>
#include <boost/shared_array.hpp>
#include <Eigen/Geometry>

#include <fovis/visual_odometry.hpp>
#include <fovis/depth_image.hpp>

namespace KSRobot
{
namespace interfaces
{

class FovisInterface : public common::VisualOdometryInterface
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef FovisInterface                      this_type;
    typedef boost::shared_ptr<this_type>        Ptr;
    typedef boost::shared_ptr<const this_type>  ConstPtr;
    
    FovisInterface();
    virtual ~FovisInterface();
    
    virtual void        RegisterToKinect(common::KinectInterface::Ptr ki);
    virtual float       GetConvergenceError();
    virtual bool        RunSingleCycle();
    
    virtual void        ReadSettings(common::ProgramOptions::Ptr po);
    
protected:
    virtual bool        CheckForKeyframe();
    virtual void        PublishKeyframeFeatures(common::VisualKeyframe::Ptr kf);
    void                InitInternal(const common::KinectInterface::CameraParameters& kinectParams, const fovis::VisualOdometryOptions& options);
    virtual void        FinishCycle();
public:
    fovis::VisualOdometry*                      mFovis;
    fovis::DepthImage*                          mDepthImage;
    
    fovis::VisualOdometryOptions                mOptions;
    fovis::Rectification*                       mRectification;
    
    bool                                        mNextCycleKeyframe;
    bool                                        mUseIcpOnFail;
    
    size_t                                      mPrevFovisReferenceFrame;
    
    common::GICP                                mGICP;
    
    CLASS_DEF_PYEXPORT;
};

#endif // FOVISINTERFACE_H

} // end namespace interfaces
} // end namespace KSRobot
