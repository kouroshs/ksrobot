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

#ifndef SLAMINTERFACE_H
#define SLAMINTERFACE_H

#include <common/Interface.h>
#include <common/VisualOdometryInterface.h>
#include <common/LoopDetector.h>
#include <tbb/concurrent_queue.h>
#include <Eigen/Geometry>
//NOTE: For now this is implemented using gtsam. maybe later extend the interface inside interfaces?

//Forward declarations in here.
namespace gtsam
{
    class NonlinearFactorGraph;
};

namespace KSRobot
{
namespace common
{

class SLAMInterface : public Interface
{
public:
    typedef SLAMInterface                       this_type;
    typedef boost::shared_ptr<this_type>        Ptr;
    typedef boost::shared_ptr<const this_type>  ConstPtr;

    SLAMInterface();
    virtual ~SLAMInterface();
    
    //THIS IS FOR TEST ONLY!
    virtual void                                ReadFromFile(const std::string& filename);
    
    virtual void                                RegisterToVO(VisualOdometryInterface::Ptr vo);
    virtual void                                RegisterToLoopDetector(LoopDetector::Ptr ld);
private:
    void                                        OnKeyframeDetected();
    void                                        OnLoopDetected(const LoopDetector::LoopClosure& lc);// called from visual odometry
protected:
    VisualOdometryInterface::Ptr                                mVO;
    int                                                         mLastKeyframe;
    boost::shared_ptr<gtsam::NonlinearFactorGraph>              mGraph;
    tbb::concurrent_bounded_queue<LoopDetector::LoopClosure>    mLoops;
    //tbb::concurrent_bounded_queue<Keyframe>                   mKeyframes;
};

} // end namespace common
} // end namespace KSRobot

#endif // SLAMINTERFACE_H
