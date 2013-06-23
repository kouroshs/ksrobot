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

#ifndef ICPINTERFACE_H
#define ICPINTERFACE_H

#include <common/VisualOdometryInterface.h>

namespace KSRobot
{
namespace interfaces
{

class ICPInterface : public common::VisualOdometryInterface
{
public:
    typedef ICPInterface                        this_type;
    typedef boost::shared_ptr<this_type>        Ptr;
    typedef boost::shared_ptr<const this_type>  ConstPtr;
    
    ICPInterface(const std::string& name);
    virtual ~ICPInterface();
    
    //virtual void                RegisterToKinect(common::KinectInterface::Ptr ki);
    virtual bool                        Converged();
    virtual float                       GetConvergenceError();
    
    virtual bool                        RunSingleCycle();
    
    inline bool                         WithNormals() const;
    void                                SetWithNormals(bool b);
private:
    class Impl;
    boost::shared_ptr<Impl>             mImpl;
    common::KinectPointCloud::ConstPtr  mLastPointCloud;
    common::KinectPointCloud            mAlignedCloud;
    
    bool                                mWithNormals;
};

bool ICPInterface::WithNormals() const
{
    return mWithNormals;
}


};
};

#endif // ICPINTERFACE_H
