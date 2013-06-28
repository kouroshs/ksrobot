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

#ifndef ENGINEINTERFACE_H
#define ENGINEINTERFACE_H

#include <common/Interface.h>
#include <common/KinectInterface.h>
#include <common/VisualOdometryInterface.h>
#include <common/SLAMInterface.h>
#include <common/PlannerInterface.h>
#include <common/MappingInterface.h>
#include <common/ExecCtrlData.h>

namespace KSRobot
{
namespace common
{

class EngineInterface : public Interface
{
public:
    typedef EngineInterface                     this_type;
    typedef boost::shared_ptr<this_type>        Ptr;
    typedef boost::shared_ptr<const this_type>  ConstPtr;

    EngineInterface();
    virtual ~EngineInterface();
    
    inline void                         SetExecutionParams(const ExecCtrlData& data);
    
    KinectInterface::Ptr                GetKinectInterface() const { return mKinect; }
    VisualOdometryInterface::Ptr        GetVisualOdometryInterface() const { return mVO; }
    SLAMInterface::Ptr                  GetSlamInterface() const { return mSLAM; }
    MappingInterface::Ptr               GetMappingInterface() const { return mMapper; }
    PlannerInterface::Ptr               GetPlannerInterface() const { return mPlanner; }
    
    virtual void                        Initialize();
protected:
    ExecCtrlData                        mExecCtrl;
    
    KinectInterface::Ptr                mKinect;
    VisualOdometryInterface::Ptr        mVO;
    SLAMInterface::Ptr                  mSLAM;
    MappingInterface::Ptr               mMapper;
    PlannerInterface::Ptr               mPlanner;
};

inline void EngineInterface::SetExecutionParams(const ExecCtrlData& data)
{
    data.CheckConsistancy();
    mExecCtrl = data;
}

} // end namespace common
} // end namespace KSRobot


#endif // ENGINEINTERFACE_H
