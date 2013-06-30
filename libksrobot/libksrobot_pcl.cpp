#include <Eigen/Geometry>
#include <common/KinectInterface.h>
#include <boost/python.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

#include <pcl/io/pcd_io.h>


using namespace KSRobot::common;

class DummyScopeClass_PCL
{
};

class DummyScopeClass_IO
{
};

void TransformPointCloud(const KinectPointCloud& in, KinectPointCloud& out, Eigen::Isometry3f iso)
{
    pcl::transformPointCloud(in, out, iso);
}


typedef pcl::VoxelGrid<KinectPointCloud::PointType>   VoxelGrid;
typedef pcl::PassThrough<KinectPointCloud::PointType>  PassThrough;

//I don't know why the hell it cant recognize the function directly
void PassThrough_FilterHelper(PassThrough& p, KinectPointCloud& pc)
{
    p.filter(pc);
}

void PassThrough_GetFilterLimitsNegativeHelper(PassThrough& p, bool& b)
{
    p.getFilterLimitsNegative(b);
}

void RemoveNaNFromPointCloud(const KinectPointCloud& in, KinectPointCloud& out)
{
    std::vector<int> dummy;
    pcl::removeNaNFromPointCloud(in, out, dummy);
}

void MergePointClouds(KinectPointCloud& inout, const KinectPointCloud& in)
{
    inout.insert(inout.end(), in.begin(), in.end());
}

void ExportPCL()
{
    using namespace boost::python;
    using namespace KSRobot::common;
    scope namespace_scope = class_<DummyScopeClass_PCL, boost::noncopyable>("pcl", no_init).
        def("transformPointCloud", make_function(TransformPointCloud))
        .staticmethod("transformPointCloud")
        .def("removeNaNFromPointCloud", make_function(RemoveNaNFromPointCloud))
        .staticmethod("removeNaNFromPointCloud")
        .def("MergePointClouds", make_function(MergePointClouds))
        .staticmethod("MergePointClouds")
    ;

    class_<DummyScopeClass_IO, boost::noncopyable>("io", no_init)
        .def("savePCDFileBinary", make_function(pcl::io::savePCDFileBinary<KinectPointCloud::PointType>))
        .staticmethod("savePCDFileBinary")
        .def("savePCDFileBinaryCompressed", make_function(pcl::io::savePCDFileBinaryCompressed<KinectPointCloud::PointType>))
        .staticmethod("savePCDFileBinaryCompressed")
        .def("savePCDFileASCII", make_function(pcl::io::savePCDFileASCII<KinectPointCloud::PointType>))
        .staticmethod("savePCDFileASCII")
        .def("loadPCDFile", make_function(pcl::io::loadPCDFile<KinectPointCloud::PointType>))
        .staticmethod("loadPCDFile")
    ;
    
    class_<VoxelGrid>("VoxelGrid", init<>())
        .def("setInputCloud", &VoxelGrid::setInputCloud)
        .def("filter", &VoxelGrid::filter)
        .def("setLeafSize", (void (VoxelGrid::*)(float,float,float))&VoxelGrid::setLeafSize);
    ;
    
    class_<PassThrough>("PassThrough", init<>())
        .def("setFilterFieldName", &PassThrough::setFilterFieldName)
        .def("getFilterFieldName", &PassThrough::getFilterFieldName)
        .def("setFilterLimits", &PassThrough::setFilterLimits)
        .def("getFilterLimits", &PassThrough::getFilterLimits)
        .def("setInputCloud", &PassThrough::setInputCloud)
        .def("filter", make_function(PassThrough_FilterHelper))
        .def("setFilterLimitsNegative", &PassThrough::setFilterLimitsNegative)
       .def("getFilterLimitsNegative", make_function(PassThrough_GetFilterLimitsNegativeHelper))
        
    ;
    
}