/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2013  Kourosh <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef OCTOMAPINTERFACE_H
#define OCTOMAPINTERFACE_H

#include <common/MappingInterface.h>
#include <common/KinectInterface.h>
#include <common/VisualOdometryInterface.h>

#include <octomap/ColorOcTree.h>
#include <octomap/Pointcloud.h>

#include <tbb/concurrent_queue.h>



namespace KSRobot
{
namespace interfaces
{
//TODO: Probably needs robot params
class OctomapInterface : public common::MappingInterface
{
public:
    typedef OctomapInterface                    this_type;
    typedef boost::shared_ptr<this_type>        Ptr;
    typedef boost::shared_ptr<const this_type>  ConstPtr;
    typedef octomap::ColorOcTree                                    OcTreeType;
    
    OctomapInterface();
    virtual ~OctomapInterface();
    
    virtual void                                                    Initialize();
    virtual bool                                                    RunSingleCycle();
    virtual void                                                    SaveToFile(const std::string& filename);
    virtual void                                                    LoadFromFile(const std::string& filename);
    virtual void                                                    ReadSettings(common::ProgramOptions::Ptr po);
    virtual void                                                    ConvertToOccupancyGrid(common::OccupancyMap::Ptr map, 
                                                                                           int center_i, int center_j, bool lock_interface = false);
    
    inline boost::shared_ptr<const OcTreeType>                      GetOctree() const;
    inline bool                                                     GetUseColor() const;
    inline void                                                     SetUseColor(bool useColor);
    inline float                                                    GetMapResolution() const;
    inline void                                                     SetMapResolution(float res);
    inline double                                                   GetMaxRange() const;
    inline void                                                     SetMaxRange(double range);
    inline bool                                                     GetApplyVoxelGrid() const;
    inline void                                                     SetApplyVoxelGrid(bool enable);
    inline float                                                    GetVoxelGridResolution() const;
    inline void                                                     SetVoxelGridResolution(float res);
    inline bool                                                     GetLazyEval() const;
    inline void                                                     SetLazyEval(bool enable);
    
    inline common::KinectPointCloud::Ptr                            GetGroundPoints() const;
    inline common::KinectPointCloud::Ptr                            GetNonGroundPoints() const;
protected:
    void                                                            ConvertInternal(common::KinectPointCloud::ConstPtr nonground,
                                                                                    common::KinectPointCloud::ConstPtr ground,
                                                                                    const Eigen::Isometry3f& transform);
    
    void                                                            UpdateMap(common::KinectPointCloud::ConstPtr pc,
                                                                              const Eigen::Vector3f& origin);
    bool                                                            CheckHeight(const OcTreeType::leaf_iterator &iter) const;
    
    bool                                                            ExtractGroundPlane(const common::KinectPointCloud& pc,
                                                                                       common::KinectPointCloud& ground,
                                                                                       common::KinectPointCloud& nonground);
protected:
    boost::shared_ptr<OcTreeType>                                   mOctree;
    
    bool                                                            mUseColor;
    float                                                           mMapResolution;
    double                                                          mMaxRange;
    
    bool                                                            mApplyVoxelGrid;
    float                                                           mVoxelGridResolution;
    
    bool                                                            mLazyEval;
    
    common::KinectPointCloud::Ptr                                   mFilteredCloud;
    common::KinectPointCloud::Ptr                                   mGroundPC;
    common::KinectPointCloud::Ptr                                   mNonGroundPC;
    
    octomap::Pointcloud                                             mOctoNonGroundPoints;
    octomap::Pointcloud                                             mOctoGroundPoints;
    
    std::vector<int>                                                mIndexes;

    //TODO: For later: Some of these filters and code should be moved to MappingInterface
    struct GroundFilter
    {
        bool                                                        Enabled;
        int                                                         Axis;
        float                                                       Distance;
        float                                                       PlaneDistance;
        float                                                       Angle;
        bool                                                        AddGroundPointsAsFreeSpace;
    };
    
    struct RobotLocalTransform
    {
        // for now only kinect local -> robot transform is available
        // which means when we get data from kinect, the forward is z+ direction
        // but we want to convert it to y+. This means a 90 degree rotation around
        // x axis.
        bool                                                        Enabled; // should be true if groundfilter is used
        float                                                       RobotHeight;
        int                                                         RobotHeightAxis;
        
        int                                                         FirstRotationAxis;
        float                                                       FirstRotationDegree;
        int                                                         SecondRotationAxis;
        float                                                       SecondRotationDegree;
    };
    
    struct PassThroughFilter
    {
        bool                                                        Enabled;
        float                                                       LimitMin;
        float                                                       LimitMax;
    };
    
    struct HeightFilter
    {
        bool                                                        Enabled;
        int                                                         Axis;
        float                                                       MaxValue;
        float                                                       MinValue;
    };
    
    //TODO: Write get&set for these
//     bool                                                            mEnableHeightFiltering;
//     int                                                             mHeightFilterAxis;
//     float                                                           mHeightFilterMaxValue;
//     float                                                           mHeightFilterMinValue;
    
    HeightFilter                                                    mHeightFilter;
    PassThroughFilter                                               mPassThrough;
    GroundFilter                                                    mGroundFilter;
    RobotLocalTransform                                             mLocalTransformInfo;
    Eigen::Affine3f                                                 mCameraTransformCached;
    
    CLASS_DEF_PYEXPORT;
};

inline boost::shared_ptr<const OctomapInterface::OcTreeType> OctomapInterface::GetOctree() const
{
    return mOctree;
}

inline bool OctomapInterface::GetUseColor() const
{
    return mUseColor;
}

inline void OctomapInterface::SetUseColor(bool useColor)
{
    mUseColor = useColor;
}

inline float OctomapInterface::GetMapResolution() const
{
    return mMapResolution;
}

inline void OctomapInterface::SetMapResolution(float res)
{
    mMapResolution = res;
}

inline double OctomapInterface::GetMaxRange() const
{
    return mMaxRange;
}

inline void OctomapInterface::SetMaxRange(double range)
{
    mMaxRange = range;
}

inline bool OctomapInterface::GetApplyVoxelGrid() const
{
    return mApplyVoxelGrid;
}

inline void OctomapInterface::SetApplyVoxelGrid(bool enable)
{
    mApplyVoxelGrid = enable;
}

inline float OctomapInterface::GetVoxelGridResolution() const
{
    return mVoxelGridResolution;
}

inline void OctomapInterface::SetVoxelGridResolution(float res)
{
    mVoxelGridResolution = res;
}

inline bool OctomapInterface::GetLazyEval() const
{
    return mLazyEval;
}

inline void OctomapInterface::SetLazyEval(bool enable)
{
    mLazyEval = enable;
}

inline common::KinectPointCloud::Ptr OctomapInterface::GetGroundPoints() const
{
    return mGroundPC;
}

inline common::KinectPointCloud::Ptr OctomapInterface::GetNonGroundPoints() const
{
    return mNonGroundPC;
}



} // end namespace interfaces
} // end namespace KSRobot
    

#endif // OCTOMAPINTERFACE_H
