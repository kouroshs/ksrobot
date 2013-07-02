#include <iostream>
#include <sstream>
#include <iomanip>
#include <common/Defenitions.h>
#include <interfaces/KinectDatasetReader.h>
#include <interfaces/FovisInterface.h>
#include <common/MappingInterface.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

#include <pcl/io/pcd_io.h>

using namespace KSRobot;
using namespace common;
using namespace interfaces;
using namespace std;

int main(int argc, char** argv)
{
    (void)argc; (void)argv;

    KinectDatasetReader::Ptr kinect(new KinectDatasetReader());
    kinect->Initialize("/windows/E/Datasets/rgbd_dataset_freiburg2_pioneer_slam/");

    FovisInterface::Ptr fovis(new FovisInterface());
    fovis->RegisterToKinect(kinect);

    ProgramOptions::Ptr po(new ProgramOptions);
    po->LoadFromFile("component-test-settings.xml");
    float angle = po->GetDouble("AxisRotAngle", 10);
    angle *= M_PI / 180;
    
    Eigen::Quaternionf quatAxis(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitX()));
    
    
    MappingInterface::Ptr map(new MappingInterface);
    map->RegisterToVO(fovis);
    map->EnableFiltering(true);
    map->SetMapResolution(0.05);
    map->SetMaxRange(4);
    map->ReInitialize();
    
    pcl::VoxelGrid<KinectPointCloud::PointType> vg;
    vg.setLeafSize(0.05, 0.05, 0.05);
    
    pcl::PassThrough<KinectPointCloud::PointType> pass;
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 4);

    const size_t max_count = 51;
    
    int curr_file_index = 0;
    int skip_counter = 0;
    const int skip_max = 5;
    
    KinectPointCloud final_pc;
    
    for(size_t i = 0; i < max_count; i++)
    {
        bool shouldskip;
        
        if( ++skip_counter == skip_max )
        {
            //DO NOT SKIP! READ THE FILE
            kinect->SetReadFiles(true);
            shouldskip = false;
            skip_counter = 0;
            curr_file_index++;
        }
        else
        {
            // SKIP CYCLE
            kinect->SetReadFiles(false);
            shouldskip = true;
        }
        kinect->RunSingleCycle();
        
        if( shouldskip )
            continue;
        
        fovis->RunSingleCycle();
        
        Eigen::Vector3f trans = fovis->GetGlobalPose().translation();
        Eigen::Quaternionf quat(fovis->GetGlobalPose().linear());
        
        KinectPointCloud::Ptr p1(new KinectPointCloud);
        vg.setInputCloud(kinect->GetPointCloud());
        vg.filter(*p1);
        
        pass.setInputCloud(p1);
        pass.filter(*p1);
        
        pcl::transformPointCloud(*p1, *p1, trans, quat);
    
        pcl::transformPointCloud(*p1, *p1, Eigen::Vector3f(0,0,0), quatAxis);
        
        final_pc.insert(final_pc.end(), p1->begin(), p1->end());
        
        map->RunSingleCycle();
        
        std::cout << "Cycle " << i << std::endl;
    }
    
    pcl::io::savePCDFileBinaryCompressed("/home/kourosh/test/pointclouds/fovis.pcd", final_pc);
    po->SaveToFile("component-test-settings.xml");
    
    map->SaveMapToFile("/home/kourosh/map.ot");
    
//     KinectDatasetReader::Ptr kinect(new KinectDatasetReader());
//     kinect->Initialize("/windows/E/Datasets/rgbd_dataset_freiburg2_pioneer_slam/");
// 
//     pcl::VoxelGrid<KinectPointCloud::PointType> vg;
//     vg.setLeafSize(0.05, 0.05, 0.05);
//     
//     pcl::PassThrough<KinectPointCloud::PointType> pass;
//     pass.setFilterFieldName("z");
//     pass.setFilterLimits(0, 4);
// 
//     const size_t max_count = 2000;
//     
//     int curr_file_index = 0;
//     int skip_counter = 0;
//     const int skip_max = 5;
//     
//     for(size_t i = 0; i < max_count; i++)
//     {
//         bool shouldskip;
//         
//         if( ++skip_counter == skip_max )
//         {
//             //DO NOT SKIP! READ THE FILE
//             kinect->SetReadFiles(true);
//             shouldskip = false;
//             skip_counter = 0;
//             curr_file_index++;
//         }
//         else
//         {
//             // SKIP CYCLE
//             kinect->SetReadFiles(false);
//             shouldskip = true;
//         }
//         kinect->RunSingleCycle();
//         
//         if( shouldskip )
//             continue;
//         
//         Eigen::Isometry3f iso = kinect->GetCurrentGroundTruth();
//         
//         Eigen::Quaternionf quat(iso.rotation());
//         Eigen::Vector3f t = iso.translation();
//         
//         KinectPointCloud::Ptr p1(new KinectPointCloud);
//         vg.setInputCloud(kinect->GetPointCloud());
//         vg.filter(*p1);
//         
//         pass.setInputCloud(p1);
//         pass.filter(*p1);
//         
//         p1->sensor_origin_ = Eigen::Vector4f(t[0], t[1], t[2], 0);
//         p1->sensor_orientation_ = quat;
//         
//         pcl::io::savePCDFileBinaryCompressed("/home/kourosh/test/pointclouds/" +
//             boost::lexical_cast<std::string>(curr_file_index) + ".pcd", *p1);
//         
//         //if( i % 10 == 0 || i == max_count - 1 )
//             std::cout << "Cycle " << i << std::endl;
//     }
    
    

//     KinectPointCloud pc;
//     
//     pcl::VoxelGrid<KinectPointCloud::PointType> vg;
//     vg.setLeafSize(0.05, 0.05, 0.05);
//     
//     pcl::PassThrough<KinectPointCloud::PointType> pass;
//     pass.setFilterFieldName("z");
//     pass.setFilterLimits(0, 4);
//     
//     const size_t max_count = 50;
//     
//     for(size_t i = 1; i < max_count; i++)
//     {
//         KinectPointCloud::Ptr loaddedPC(new KinectPointCloud);
//         pcl::io::loadPCDFile("/home/kourosh/test/pointclouds/" +
//             boost::lexical_cast<std::string>(i) + ".pcd", *loaddedPC);
//         
//         KinectPointCloud p1;
//         vg.setInputCloud(loaddedPC);
//         vg.filter(p1);
//         
//         *loaddedPC = p1;
//         
//         pass.setInputCloud(loaddedPC);
//         p1.clear();
//         pass.filter(p1);
//         
//         pcl::transformPointCloud(p1, p1, Eigen::Vector3f(p1.sensor_origin_[0], p1.sensor_origin_[1], p1.sensor_origin_[2]),
//             p1.sensor_orientation_);
//         
//         pc.insert(pc.end(), p1.begin(), p1.end());
//         if( i % 10 == 0 || i == max_count - 1 )
//             std::cout << "Cycle " << i << std::endl;
//     }
//     
//     pcl::io::savePCDFileBinary("/home/kourosh/test/pointclouds/final.pcd", pc);
    
    return 0;
}
