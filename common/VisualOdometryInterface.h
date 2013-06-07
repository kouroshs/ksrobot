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

namespace KSRobot
{
namespace common
{

//TODO: ADD OTHER METHODS
class VisualOdometryInterface : public Interface
{
public:
    typedef VisualOdometryInterface             this_type;
    typedef boost::shared_ptr<this_type>        Ptr;
    typedef boost::shared_ptr<const this_type>  ConstPtr;

    VisualOdometryInterface(ProgramOptions::Ptr po, const std::string& name);
    virtual ~VisualOdometryInterface();
    

    virtual void        RegisterToKinect(KinectInterface::Ptr ki) = 0;    
};

} // end namespace common
} // end namespace KSRobot

#endif // VISUALODOMETRYINTERFACE_H
