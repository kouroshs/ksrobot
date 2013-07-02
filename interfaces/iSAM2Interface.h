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

#ifndef ISAM2INTERFACE_H
#define ISAM2INTERFACE_H

#include <common/SLAMInterface.h>

//Forward declaration:
namespace gtsam
{
    class ISAM2;
};

namespace KSRobot
{
namespace interfaces
{

class iSAM2Interface : public common::SLAMInterface
{
public:
    iSAM2Interface();
    virtual ~iSAM2Interface();
    
    
    virtual void ReadSettings(common::ProgramOptions::Ptr po);
    virtual void WriteSettings(common::ProgramOptions::Ptr po);
    
    
    virtual bool RunSingleCycle();
private:
    gtsam::ISAM2*                               mISAM2;
};

};
};

#endif // ISAM2INTERFACE_H
