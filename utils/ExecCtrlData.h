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

#ifndef EXECUTIONCONTROL_H
#define EXECUTIONCONTROL_H

#include <string>

namespace KSRobot
{
namespace utils
{

class ExecCtrlData
{
public:
    struct
    {
        bool            GetFromDevice;
        std::string     SourceDevice;
        std::string     SourceDir;
    } Kinect;
    
    struct
    {
        bool            Enable;
    } Fovis;
    
    struct
    {
        bool            Enable;
    } iSAM;
    
    struct
    {
        bool            Enable;
    } OctoMap;
    
    struct
    {
        bool            Enable;
    } OMPL;
    
    struct
    {
        bool            Enable;
        int             Port;
        std::string     Address;
    } Comm;
    
    struct
    {
        bool            ViewRGBD;
        bool            ViewPCL;
        bool            ViewFovis;
        bool            ViewOctoMap;
        bool            ViewOMPL;
    } GUI;
    
    void                CheckConsistancy();
    
    void                EnableFovis(bool enable);
    void                EnableSAM(bool enable);
    void                EnableOctoMap(bool enable);
    void                EnableOMPL(bool enable);
    void                EnableComm(bool enable);
};

} // end namespace utils
} // end namespace KSRobot

#endif // EXECUTIONCONTROL_H
