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
#include <common/LoopDetectorInterface.h>
#include <tbb/concurrent_queue.h>
#include <Eigen/Geometry>
//NOTE: For now this is implemented using gtsam. maybe later extend the interface inside interfaces?

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
    
    void                                        RegisterToVO(VisualOdometryInterface::Ptr vo);
    void                                        RegisterToLoopDetector(LoopDetectorInterface::Ptr ld);
    virtual bool                                RunSingleCycle();
private:
    void                                        OnKeyframeDetected(const VisualKeyframe::Ptr kf);
    void                                        OnLoopDetected(const LoopDetectorInterface::LoopClosure& lc);// called from visual odometry
protected:
    virtual void                                AddKeyframe(const VisualKeyframe::Ptr kf) { (void)kf;}
    virtual void                                AddLoopClosure(const LoopDetectorInterface::LoopClosure& lc) { (void)lc;}
    virtual void                                Update() {;} // called if AddKeyframe and AddLoopClosure need finilization code to perform.
    virtual void                                FinishCycle();
protected:
    struct SLAMDataArrival
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        LoopDetectorInterface::LoopClosure          Loop;
        VisualKeyframe::Ptr                         Keyframe;
        bool                                        IsLoopClosure;
    };
    
    VisualOdometryInterface::Ptr                                        mVO;
    int                                                                 mLastKeyframe;
    tbb::concurrent_queue<SLAMDataArrival>                              mUnprocessedData;
    
    CLASS_DEF_PYEXPORT;
};

} // end namespace common
} // end namespace KSRobot

#endif // SLAMINTERFACE_H
