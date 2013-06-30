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

// void Callback(const pcl::PointCloud<pcl::PointXYZ>::Ptr& p)
// {
//     std::cout << "CALLBACK CALLED!\n" << std::flush;
// }

int main(int argc, char** argv)
{
    (void)argc; (void)argv;
    ProgramOptions::Ptr settings(new ProgramOptions());
    settings->LoadFromFile("settings.xml");
    
    KinectDatasetReader::Ptr kinect(new KinectDatasetReader());
//     FovisInterface::Ptr fovis(new FovisInterface());
//     fovis->ReadSettings(settings->StartNode("FovisInterface"));

    kinect->Initialize("/windows/E/Datasets/rgbd_dataset_freiburg2_pioneer_slam/");
//     fovis->RegisterToKinect(kinect);

    std::vector<int> dummyVect;

    KinectPointCloud::Ptr kpc(new KinectPointCloud);
    
    for(int i = 0; i < 1; i++)
    {
        kinect->RunSingleCycle();
        KinectPointCloud::ConstPtr pc = kinect->GetPointCloud();
        KinectPointCloud::Ptr filteredPC(new KinectPointCloud(*pc));
        
//         pcl::VoxelGrid<KinectPointCloud::PointType> vg;
//         vg.setLeafSize(0.02, 0.02, 0.02);
//         vg.setInputCloud(pc);
//         vg.filter(*filteredPC);
//         
        
        
//         pcl::PassThrough<KinectPointCloud::PointType> passFilter;
//         passFilter.setFilterLimits(0, 3);
//         passFilter.setFilterFieldName("z");
//         passFilter.setInputCloud(filteredPC);
//         passFilter.filter(*filteredPC);

        filteredPC->sensor_orientation_ = kinect->GetCurrentGroundTruth().linear().cast<float>();
        Eigen::Vector3f v = kinect->GetCurrentGroundTruth().translation().cast<float>();
        filteredPC->sensor_origin_ = Eigen::Vector4f(v[0], v[1], v[2], 1);
        
        pcl::io::savePCDFileBinaryCompressed("/home/kourosh/test/" +  boost::lexical_cast<std::string>(i) + ".pcd", *filteredPC);
        
//        pcl::transformPointCloud(*filteredPC, *filteredPC, kinect->GetCurrentGroundTruth().cast<float>());
//        kpc->insert(kpc->end(), filteredPC->begin(), filteredPC->end());
        
//         vg.setInputCloud(kpc);
//         vg.filter(*kpc);
        
        std::cout << "Cycle " << i << std::endl; 
//         std::cout << "RGB: " << kinect->GetCurrentRgbFileName() << std::endl;
//         std::cout << "DEPTH: " << kinect->GetCurrentDepthFileName() << std::endl;
        std::cout << std::flush;
    }
    //pcl::io::savePCDFileBinary("/home/kourosh/test/pointclouds/all.pcd", *kpc);
    settings->SaveToFile("settings.xml");
    return 0;
}
