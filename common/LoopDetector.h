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

#ifndef LOOPDETECTOR_H
#define LOOPDETECTOR_H

#include <common/Interface.h>
#include <common/VisualOdometryInterface.h>
#include <boost/signals2.hpp>

namespace KSRobot
{
namespace common
{

class LoopDetector : public Interface
{
public:
    class LoopClosure
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        Eigen::Isometry3d               Transform; // Transform from 1 to 2
        int                             Cycle1;
        int                             Cycle2;
    };
    
    typedef LoopDetector this_type;
    typedef boost::shared_ptr<this_type>                Ptr;
    typedef boost::shared_ptr<const this_type>          ConstPtr;
    
    LoopDetector();
    virtual ~LoopDetector();
    
    virtual void                                RegisterToVO(VisualOdometryInterface::Ptr vo);
    boost::signals2::connection                 RegisterLoopReceiver(boost::function<void(const LoopClosure& lc)> fn);
    
protected:
    VisualOdometryInterface::Ptr                mVO;
    boost::signals2::signal<void(const LoopClosure& lc)>  mOnLoopDetected;
};

};
};

#endif // LOOPDETECTOR_H
